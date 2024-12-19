import rclpy
import os
import datetime
import threading
import shutil
from rclpy.time import Duration, Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, QoSPresetProfiles, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_srvs.srv import SetBool
from bagtube_msgs.action import RecordBag, PlayBag
from bagtube_msgs.srv import ControlPlayback, GetBagList, EditBag
from bagtube_msgs.msg import BagInfo
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from rosbag2_py import StorageOptions, Player, Info, SequentialReader, ReadOrder, ReadOrderSortBy, StorageFilter

from bagtube_rosbag2_py import Player, PlayOptions, RecordOptions, Recorder, init_rclcpp, shutdown_rclcpp
from rosbag2_py import get_registered_writers


class BagtubeServer(Node):
    STAMP_FORMAT = '%Y_%m_%d-%H_%M_%S.%f'
    STAMP_FORMAT_LENGTH = len(datetime.datetime.utcnow().strftime(STAMP_FORMAT))
    DEFAULT_WRITER = 'sqlite3' if 'sqlite3' in get_registered_writers() else get_registered_writers()[0]


    def __init__(self):
        super().__init__('bagtube_server')
        pipes = self.declare_parameter('pipes', rclpy.Parameter.Type.STRING_ARRAY).get_parameter_value().string_array_value

        self.declare_parameter('bag_dir_path', os.path.expanduser('~/.ros/bagtube'))
        self.declare_parameter('max_total_size_gb', 0.0, ParameterDescriptor(description=
                                                         'Maximum total size of all bags in GB. 0 for unlimited. If exceeded, oldest bags will be deleted. Note:'
                                                         ' this is checked after a new bag is recorded, so the actual size may exceed this limit by a small amount.'))
        self.declare_parameter('initial_wait_for_publishers_timeout', 10.0)

        self.bag_dir_path = self.get_parameter('bag_dir_path').get_parameter_value().string_value
        self.bag_dir_path = os.path.abspath(self.bag_dir_path)
        os.makedirs(self.bag_dir_path, exist_ok=True)
        self.max_total_size_gb = self.get_parameter('max_total_size_gb').get_parameter_value().double_value
        self.initial_wait_for_publishers_timeout = self.get_parameter('initial_wait_for_publishers_timeout').get_parameter_value().double_value

        self.livestream_enabled = False
        self.recording_bag = False
        self.snapshot_reader = None
        self.snapshot_uri = None

        # TODO: create Pipe class instead of using many dictionaries
        self.pipe_publishers = {}
        self.pipe_subscribers = {}
        self.topic_pipes = {}
        self.input_nodes_running_services = {}
        self.topic_remappings = {}
        self.pipe_transient_local_last_messages = {}
        self.pipe_qos_profiles = {}
        self.input_nodes_running_services_lock = threading.Lock()

        self.init_time = self.get_clock().now()
        for pipe in pipes:
            type_str = self.declare_parameter(f'{pipe}.type', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            input_topic = self.declare_parameter(f'{pipe}.input_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            output_topic = self.declare_parameter(f'{pipe}.output_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value

            self.topic_remappings[input_topic] = output_topic
            self.topic_pipes[input_topic] = pipe

            msg_type = self.load_message_type(type_str)  # TODO: get type automatically from publishers
            if msg_type:
                qos = self.get_qos_profile_for_topic(input_topic)
                self.pipe_qos_profiles[pipe] = qos
                self.pipe_publishers[pipe] = self.create_publisher(msg_type, output_topic, qos)
                self.pipe_subscribers[pipe] = self.create_subscription(
                    msg_type,
                    input_topic,
                    lambda msg, pipe=pipe: self.pipe_callback(msg, pipe),
                    qos,
                )
            else:
                self.get_logger().error(f"Invalid message type '{type_str}' for pipe '{pipe}'")

        self.livestream_service = self.create_service(SetBool, 'enable_livestream', self.livestream_callback)
        self.toggle_input_nodes_services = {}
        toggle_input_nodes_services_param = self.declare_parameter('toggle_input_nodes_running_services', rclpy.Parameter.Type.STRING_ARRAY)
        toggle_input_nodes_services = toggle_input_nodes_services_param.get_parameter_value().string_array_value

        self.sync_service_callback_group = MutuallyExclusiveCallbackGroup()
        for service_name in toggle_input_nodes_services:
            self.toggle_input_nodes_services[service_name] = self.create_client(SetBool, service_name, callback_group=self.sync_service_callback_group)

        self.get_bag_list_service = self.create_service(GetBagList, 'get_bag_list', self.get_bag_list_callback)
        self.edit_bag_service = self.create_service(EditBag, 'edit_bag', self.edit_bag_callback)

        self.record_server = ActionServer(
            self,
            RecordBag,
            'record_bag',
            self.record_bag_callback,
            goal_callback=self.record_goal_callback,
            cancel_callback=self.record_cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.canceled_recordings = []  # cannot be a set() because UUIDs are not hashable
        self.recordings_queue = []
        self.recordings_queue_condition = threading.Condition()

        self.play_control_lock = threading.Lock()
        self.clock_subscriber = self.create_subscription(Clock, 'clock', self.clock_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.play_server = ActionServer(
            self,
            PlayBag,
            'play_bag',
            self.play_bag_callback,
            goal_callback=self.play_goal_callback,
            cancel_callback=self.play_cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.control_playback_service = self.create_service(ControlPlayback, 'control_playback', self.control_playback_callback)

        self.init_timer = self.create_timer(0.0, self.init_timer_callback)
        init_rclcpp()

    def init_timer_callback(self):
        self.init_timer.cancel()
        self.toggle_input_nodes_running()

    def livestream_callback(self, request, response):
        with self.input_nodes_running_services_lock:
            self.livestream_enabled = request.data
        if request.data:
            for pipe, msg in self.pipe_transient_local_last_messages.items():
                self.pipe_publishers[pipe].publish(msg)
        self.toggle_input_nodes_running()

        response.success = True
        response.message = f"Livestream {'enabled' if request.data else 'disabled'}"
        return response

    def toggle_input_nodes_running(self):
        with self.input_nodes_running_services_lock:
            for service_name, service_client in self.toggle_input_nodes_services.items():
                if service_client.wait_for_service(timeout_sec=1.0):
                    client_request = SetBool.Request()
                    client_request.data = self.livestream_enabled or self.recording_bag
                    result = service_client.call(client_request)
                    if result is not None:
                        self.get_logger().info(f"{service_name} service call result: {result.message}")
                    else:
                        self.get_logger().warn(f"{service_name} service call failed: {result.exception()}")


    def load_message_type(self, type_str):
        try:
            module_names = type_str.split('/')
            module_name = '.'.join(module_names[:-1])
            class_name = module_names[-1]
            module = __import__(module_name, fromlist=[class_name])
            msg_type = getattr(module, class_name)
            return msg_type
        except (ValueError, ImportError, AttributeError):
            return None

    def get_qos_profile_for_topic(self, topic_name):
        qos_profile = QoSPresetProfiles.get_from_short_key('sensor_data')
        reliability_reliable_endpoints_count = 0
        durability_transient_local_endpoints_count = 0

        pubs_info = self.get_publishers_info_by_topic(topic_name)
        publishers_count = len(pubs_info)
        while rclpy.ok() and publishers_count == 0 and self.get_clock().now() - self.init_time < Duration(seconds=self.initial_wait_for_publishers_timeout):
            self.get_logger().warn(f"No publishers found for topic '{topic_name}'. Waiting for publishers...")
            self.get_clock().sleep_for(Duration(seconds=1))
            pubs_info = self.get_publishers_info_by_topic(topic_name)
            publishers_count = len(pubs_info)
            if publishers_count > 0:
                self.get_logger().info(f"Publishers found for topic '{topic_name}', done waiting")

        if publishers_count == 0:
            self.get_logger().warn(f"No publishers found for topic '{topic_name}'. Using default QoS profile")
            return qos_profile

        for info in pubs_info:
            if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
                reliability_reliable_endpoints_count += 1
            if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
                durability_transient_local_endpoints_count += 1

        # If all endpoints are reliable, ask for reliable
        if reliability_reliable_endpoints_count == publishers_count:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            if reliability_reliable_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSReliabilityPolicy.RELIABLE. Falling back to '
                    'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                    'to all publishers'
                )
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # If all endpoints are transient_local, ask for transient_local
        if durability_transient_local_endpoints_count == publishers_count:
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        else:
            if durability_transient_local_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                    'QoSDurabilityPolicy.VOLATILE as it will connect '
                    'to all publishers'
                )
            qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        return qos_profile

    def pipe_callback(self, msg, pipe):
        if self.pipe_qos_profiles[pipe].durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
            self.pipe_transient_local_last_messages[pipe] = msg
        if self.livestream_enabled:
            self.pipe_publishers[pipe].publish(msg)

    def get_bag_list_callback(self, request: GetBagList.Request, response: GetBagList.Response):
        # return a list of bag directories whose names are in format '{name}-%Y_%m_%d-%H_%M_%S.%f'
        prefix = request.name + '-'
        required_len = len(prefix) + BagtubeServer.STAMP_FORMAT_LENGTH
        filter = (lambda dirname: len(dirname) >= required_len and dirname[-BagtubeServer.STAMP_FORMAT_LENGTH - 1] == '-')\
                    if len(request.name) == 0 else\
                 (lambda dirname: dirname.startswith(prefix) and len(dirname) == required_len)
        bag_dirnames = []
        for dirname in os.listdir(self.bag_dir_path):
            if os.path.isdir(os.path.join(self.bag_dir_path, dirname)) and filter(dirname):
                bag_dirnames.append(dirname)

        # extract the timestamp from the directory name
        for dirname in bag_dirnames:
            nsec = int(datetime.datetime.strptime(dirname[-BagtubeServer.STAMP_FORMAT_LENGTH:], BagtubeServer.STAMP_FORMAT)
                       .replace(tzinfo=datetime.timezone.utc).timestamp()*1e9)
            bag = BagInfo()
            bag.name = dirname[:-BagtubeServer.STAMP_FORMAT_LENGTH - 1]
            bag.stamp = Time(nanoseconds=nsec).to_msg()

            # get the duration of the bag
            try:
                info = Info()
                metadata = info.read_metadata(os.path.join(self.bag_dir_path, dirname), BagtubeServer.DEFAULT_WRITER)
                bag.duration = metadata.duration.nanoseconds / 1e9

                response.bags.append(bag)
            except Exception as e:
                self.get_logger().error(f"Error reading metadata for bag '{dirname}': {e}")

        # sort the bags by timestamp
        response.bags.sort(key=lambda bag: bag.stamp.sec + bag.stamp.nanosec/1e9)
        return response

    def edit_bag_callback(self, request: EditBag.Request, response: EditBag.Response):
        sec, nsec = Time.from_msg(request.stamp).seconds_nanoseconds()
        date_string = datetime.datetime.utcfromtimestamp(sec + nsec/1e9).strftime(BagtubeServer.STAMP_FORMAT)
        bag_name = request.name + '-' + date_string
        # rename the bag directory
        if request.operation == EditBag.Request.RENAME:
            old_bag_name = bag_name
            old_bag_path = os.path.join(self.bag_dir_path, old_bag_name)
            new_bag_name = request.new_name + '-' + date_string
            new_bag_path = os.path.join(self.bag_dir_path, new_bag_name)
            if os.path.exists(old_bag_path):
                os.rename(old_bag_path, new_bag_path)
                response.success = True
                response.message = f"Bag renamed from '{old_bag_name}' to '{new_bag_name}'"
            else:
                response.success = False
                response.message = f"Bag '{old_bag_name}' does not exist"
        elif request.operation == EditBag.Request.DELETE:
            bag_name = bag_name
            bag_path = os.path.join(self.bag_dir_path, bag_name)
            if os.path.exists(bag_path):
                shutil.rmtree(bag_path)
                response.success = True
                response.message = f"Bag deleted: '{bag_name}'"
            else:
                response.success = False
                response.message = f"Bag '{bag_name}' does not exist"
        return response

    #############################
    # Record bag action
    def record_goal_callback(self, goal: RecordBag.Goal):
        self.get_logger().info(f"Received record goal: {goal.name} {goal.max_duration}")
        return GoalResponse.ACCEPT

    def record_cancel_callback(self, goal_handle):
        self.get_logger().info(f"Record goal {goal_handle.goal_id} canceled")
        with self.recordings_queue_condition:
            if goal_handle.is_active:
                self.canceled_recordings.append(goal_handle.goal_id)
                self.recordings_queue_condition.notify_all()
        return CancelResponse.ACCEPT

    def record_bag_callback(self, goal_handle):
        record_bag_goal = goal_handle.request

        with (self.recordings_queue_condition):
            self.recordings_queue.append(goal_handle.goal_id)
            if len(self.recordings_queue) > 1:
                self.get_logger().info(f"RecordBag goal {goal_handle.goal_id} queued "
                                       f"(multiple recordings at a time are not supported currently)")
                feedback = RecordBag.Feedback()
                feedback.queued = True
                goal_handle.publish_feedback(feedback)

                while rclpy.ok() and not (goal_handle.is_cancel_requested or goal_handle.goal_id in self.canceled_recordings)\
                      and self.recordings_queue[0] != goal_handle.goal_id:
                    self.recordings_queue_condition.wait()

        bag_name = record_bag_goal.name + '-' + datetime.datetime.utcnow().strftime(BagtubeServer.STAMP_FORMAT)
        bag_path = os.path.join(self.bag_dir_path, bag_name)

        self.get_logger().info(f"Recording bag '{bag_path}' for {record_bag_goal.max_duration} seconds")

        storage_options = StorageOptions(
            uri=bag_path,
            storage_id=BagtubeServer.DEFAULT_WRITER,
        )
        record_options = RecordOptions()
        record_options.topics = list(self.topic_remappings.keys())
        record_options.is_discovery_disabled = False
        record_options.topic_polling_interval = datetime.timedelta(milliseconds=100)

        recorder = Recorder()

        with self.input_nodes_running_services_lock:
            self.recording_bag = True
        self.toggle_input_nodes_running()

        record_thread = threading.Thread(
            target=recorder.record,
            args=(storage_options, record_options),
            daemon=True)

        max_duration = Duration(seconds=record_bag_goal.max_duration)
        start = self.get_clock().now()
        record_thread.start()
        while rclpy.ok() and not goal_handle.is_cancel_requested and (max_duration.nanoseconds == 0 or self.get_clock().now() - start < max_duration):
            feedback = RecordBag.Feedback()
            feedback.current_duration = (self.get_clock().now() - start).nanoseconds / 1e9
            goal_handle.publish_feedback(feedback)
            self.get_clock().sleep_for(Duration(seconds=0.1))
        recorder.cancel()
        record_thread.join()
        self.get_logger().info(f"Recording finished. Bag saved at: {bag_path}")

        with self.input_nodes_running_services_lock:
            self.recording_bag = False
        self.toggle_input_nodes_running()

        result = RecordBag.Result()

        gb_to_bytes = 1024 * 1024 * 1024
        if self.max_total_size_gb > 0.0 and Info().read_metadata(bag_path, BagtubeServer.DEFAULT_WRITER).bag_size > self.max_total_size_gb*gb_to_bytes:
            self.get_logger().info(f"Recorded bag size exceeds limit of {self.max_total_size_gb} GB, deleting")
            shutil.rmtree(bag_path)
        self.limit_total_size()

        if os.path.exists(bag_path):
            metadata = Info().read_metadata(bag_path, BagtubeServer.DEFAULT_WRITER)
            if metadata.message_count == 0:
                shutil.rmtree(bag_path)
                result.success = False
                result.message = f"Bag is empty"
                self.get_logger().info(f"Bag is empty, deleting")
            else:
                result.bag.name = record_bag_goal.name
                nsec = int(datetime.datetime.strptime(bag_name[-BagtubeServer.STAMP_FORMAT_LENGTH:], BagtubeServer.STAMP_FORMAT)
                           .replace(tzinfo=datetime.timezone.utc).timestamp()*1e9)
                result.bag.stamp = Time(nanoseconds=nsec).to_msg()
                result.bag.duration = metadata.duration.nanoseconds / 1e9
                result.success = True
        else:
            result.success = False
            result.message = f"Bag file does not exist (it was probably deleted due to size limit)"
            self.get_logger().info(f"Bag file does not exist (it was probably deleted due to size limit)")

        with self.recordings_queue_condition:
            self.recordings_queue.remove(goal_handle.goal_id)
            self.recordings_queue_condition.notify_all()

            # must be inside the condition lock to prevent adding the uuid to canceled_recordings after the job is done
            try:
                self.canceled_recordings.remove(goal_handle.goal_id)
            except ValueError:
                pass
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.succeed()
        return result

    def limit_total_size(self):
        # limit the total size of all bags
        if self.max_total_size_gb > 0.0:
            filter = (lambda dirname: len(dirname) > BagtubeServer.STAMP_FORMAT_LENGTH and dirname[-BagtubeServer.STAMP_FORMAT_LENGTH - 1] == '-')
            bag_dirnames = []
            for dirname in os.listdir(self.bag_dir_path):
                if os.path.isdir(os.path.join(self.bag_dir_path, dirname)) and filter(dirname):
                    bag_dirnames.append(dirname)
            bag_dirnames.sort(key=lambda dirname: dirname[-BagtubeServer.STAMP_FORMAT_LENGTH:], reverse=True)
            total_size = 0
            for dirname in bag_dirnames:
                total_size += Info().read_metadata(os.path.join(self.bag_dir_path, dirname), BagtubeServer.DEFAULT_WRITER).bag_size

            gb_to_bytes = 1024 * 1024 * 1024
            if total_size > self.max_total_size_gb * gb_to_bytes:
                for dirname in reversed(bag_dirnames):
                    if total_size <= self.max_total_size_gb * gb_to_bytes:
                        break
                    self.get_logger().info(f"Deleting bag '{dirname}' to limit total size to {self.max_total_size_gb} GB")
                    total_size -= Info().read_metadata(os.path.join(self.bag_dir_path, dirname), BagtubeServer.DEFAULT_WRITER).bag_size
                    shutil.rmtree(os.path.join(self.bag_dir_path, dirname))

    #############################
    # Play bag action
    def play_goal_callback(self, goal: RecordBag.Goal):
        self.get_logger().info(f"Received play goal: {goal.name} {goal.stamp.sec} {goal.start_offset}")
        return GoalResponse.ACCEPT

    def play_cancel_callback(self, goal_handle):
        self.get_logger().info(f"Play goal {goal_handle.goal_id} canceled")
        return CancelResponse.ACCEPT

    def clock_callback(self, msg):
        with self.play_control_lock:
            self.playback_stamp = Time.from_msg(msg.clock)

    def play_bag_callback(self, goal_handle):
        play_bag_goal: PlayBag.Goal = goal_handle.request

        sec, nsec = Time.from_msg(play_bag_goal.stamp).seconds_nanoseconds()
        bag_name = play_bag_goal.name + '-' + datetime.datetime.utcfromtimestamp(sec + nsec/1e9).strftime(BagtubeServer.STAMP_FORMAT)
        bag_path = os.path.join(self.bag_dir_path, bag_name)

        info = Info()
        metadata = info.read_metadata(bag_path, BagtubeServer.DEFAULT_WRITER)

        self.get_logger().info(f"Playing bag '{bag_path}' with duration {metadata.duration} seconds")

        storage_options = StorageOptions(
            uri=bag_path,
            storage_id=BagtubeServer.DEFAULT_WRITER,
        )

        topic_remapping = ['--ros-args']
        for key in self.topic_remappings:
            topic_remapping.append('--remap')
            topic_remapping.append(f'{key}:={self.topic_remappings[key]}')
        play_options = PlayOptions()
        play_options.topic_remapping_options = topic_remapping
        play_options.topics_to_filter = list(self.topic_remappings.keys())
        play_options.disable_keyboard_controls = True
        play_options.clock_publish_frequency = 100
        play_options.start_offset = play_bag_goal.start_offset
        play_options.rate = play_bag_goal.rate

        with self.play_control_lock:
            self.playback_start_time = Time(nanoseconds=metadata.starting_time.nanoseconds, clock_type=rclpy.clock.ClockType.ROS_TIME)
            self.playback_stamp = self.playback_start_time + Duration(seconds=play_bag_goal.start_offset)
            self.player = Player(storage_options, play_options)

        play_thread = threading.Thread(
            target=self.player.play,
            daemon=True)

        play_thread.start()
        feedback = PlayBag.Feedback()
        feedback.current_offset = -1.0  # make sure the first feedback is published
        while rclpy.ok() and not goal_handle.is_cancel_requested and not self.player.has_finished():
            with self.play_control_lock:
                current_offset = (self.playback_stamp - self.playback_start_time).nanoseconds / 1e9
            if feedback.current_offset != current_offset:
                feedback.current_offset = current_offset
                goal_handle.publish_feedback(feedback)
            self.get_clock().sleep_for(Duration(seconds=0.1))
        with self.play_control_lock:
            self.player.cancel()
        play_thread.join()

        result = PlayBag.Result()
        with self.play_control_lock:
            result.stop_offset = (self.playback_stamp - self.playback_start_time).nanoseconds / 1e9
            self.player = None
        self.get_logger().info(f"Playing finished after {result.stop_offset} seconds")
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def control_playback_callback(self, request: ControlPlayback.Request, response):
        additional_message = ''
        if request.command == ControlPlayback.Request.SEEK and (request.snapshot_bag_stamp.sec != 0 or request.snapshot_bag_stamp.nanosec != 0):
            # get the snapshot bag path
            sec, nsec = Time.from_msg(request.snapshot_bag_stamp).seconds_nanoseconds()
            bag_name = request.snapshot_bag_name + '-' + datetime.datetime.utcfromtimestamp(sec + nsec/1e9).strftime(BagtubeServer.STAMP_FORMAT)
            bag_path = os.path.join(self.bag_dir_path, bag_name)
            if not os.path.exists(bag_path):
                additional_message = "; snapshot bag does not exist, so no snapshot was published"
            else:
                # open the snapshot bag and find the last messages of each topic before the requested timestamp
                info = Info()
                metadata = info.read_metadata(bag_path, BagtubeServer.DEFAULT_WRITER)
                snapshot_stamp = metadata.starting_time + Duration(seconds=request.offset)
                snapshot_msgs = self.get_last_messages_before_timestamp(bag_path, snapshot_stamp)
                for topic in snapshot_msgs:
                    self.pipe_publishers[self.topic_pipes[topic]].publish(snapshot_msgs[topic])
                additional_message = f"; sent snapshot for {len(snapshot_msgs)} topics"

        with self.play_control_lock:
            if not hasattr(self, 'player') or self.player is None or self.player.has_finished():
                self.get_logger().error(f"Playback control requested but no bag is playing" + additional_message)
                response.success = False
                response.message = f"Playback control requested but no bag is playing" + additional_message
                return response
            if request.command == ControlPlayback.Request.PAUSE:
                self.player.pause()
                response.current_offset = (self.playback_stamp - self.playback_start_time).nanoseconds / 1e9
            elif request.command == ControlPlayback.Request.RESUME:
                self.player.resume()
            elif request.command == ControlPlayback.Request.SEEK:
                self.player.seek((self.playback_start_time + rclpy.time.Duration(seconds=request.offset)).nanoseconds)
            elif request.command == ControlPlayback.Request.SET_RATE:
                self.player.set_rate(request.rate)
            else:
                self.get_logger().error(f"Invalid playback command: {request.command}" + additional_message)
                response.success = False
                response.message = f"Invalid playback command: {request.command}" + additional_message
                return response
        response.success = True
        response.message = f"Playback command '{request.command}' successful" + additional_message
        return response

    def get_last_messages_before_timestamp(self, bag_path, target_timestamp):
        last_messages = {}
        if self.snapshot_uri != bag_path:
            # Open the bag
            reader = SequentialReader()
            reader.open_uri(bag_path)

            # Set the read order to reverse because we want the last message before the target timestamp
            read_order = ReadOrder()
            read_order.sort_by = ReadOrderSortBy.ReceivedTimestamp
            read_order.reverse = True
            reader.set_read_order(read_order)

            topics_meta_list = reader.get_all_topics_and_types()
            topics_meta = {topic.name : topic for topic in topics_meta_list}
            filtered_topics = [topic for topic in self.topic_remappings if topic in topics_meta]
            filter = StorageFilter()
            filter.topics = filtered_topics
            reader.set_filter(filter)

            self.snapshot_uri = bag_path
            self.snapshot_reader = reader
            self.snapshot_filtered_topics = filtered_topics
            self.snapshot_topics_meta = topics_meta
        else:
            reader = self.snapshot_reader
            topics_meta = self.snapshot_topics_meta
            filtered_topics = self.snapshot_filtered_topics

        reader.seek(target_timestamp.nanoseconds)

        # Iterate through the messages in the bag
        while reader.has_next():
            topic, data, _ = reader.read_next()  # read_next() returns (type_map index, serialized_message, timestamp)
            topic_meta = topics_meta[topic]

            if not (topic_meta.name in last_messages):
                msg_type = get_message(topic_meta.type)
                msg = deserialize_message(data, msg_type)
                last_messages[topic] = msg
                if len(last_messages) == len(filtered_topics):
                    break

        return last_messages

def main(args=None):
    rclpy.init(args=args)
    node = BagtubeServer()
    executor = MultiThreadedExecutor(12)
    executor.add_node(node)
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        shutdown_rclcpp()
        node.destroy_node()

if __name__ == '__main__':
    main()
