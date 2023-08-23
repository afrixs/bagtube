import rclpy
import os
import datetime
import threading
from rclpy.time import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_srvs.srv import SetBool
from bagtube_msgs.action import RecordBag

from rosbag2_py import StorageOptions, RecordOptions, Recorder, Player
from rosbag2_py import get_registered_compressors
from rosbag2_py import get_registered_serializers
from rosbag2_py import get_registered_writers


class BagtubeServer(Node):
    def __init__(self):
        super().__init__('bagtube_server')
        pipes = self.declare_parameter('pipes', rclpy.Parameter.Type.STRING_ARRAY).get_parameter_value().string_array_value

        self.declare_parameter('bag_dir_path', os.path.expanduser('~/.ros/bagtube'))
        self.bag_dir_path = self.get_parameter('bag_dir_path').get_parameter_value().string_value
        self.bag_dir_path = os.path.abspath(self.bag_dir_path)
        os.makedirs(self.bag_dir_path, exist_ok=True)

        self.livestream_enabled = False
        self.recording_bag = False

        self.pipe_publishers = {}
        self.pipe_subscribers = {}
        self.input_nodes_running_services = {}
        self.topics = []
        self.input_nodes_running_services_lock = threading.Lock()

        for pipe in pipes:
            type_str = self.declare_parameter(f'{pipe}.type', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            input_topic = self.declare_parameter(f'{pipe}.input_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            output_topic = self.declare_parameter(f'{pipe}.output_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value

            self.topics.append(input_topic)

            msg_type = self.load_message_type(type_str)
            if msg_type:
                self.pipe_publishers[pipe] = self.create_publisher(msg_type, output_topic, 10)
                qos = self.get_qos_profile_for_msg_type(msg_type)
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

        self.action_server = ActionServer(
            self,
            RecordBag,
            'record_bag',
            self.record_bag_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.init_timer = self.create_timer(0.0, self.init_timer_callback)

    def init_timer_callback(self):
        self.init_timer.cancel()
        self.toggle_input_nodes_running()

    def livestream_callback(self, request, response):
        with self.input_nodes_running_services_lock:
            self.livestream_enabled = request.data
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

    def get_qos_profile_for_msg_type(self, msg_type):
        # Define custom QoS profiles for specific message types if needed
        # For now, using a sensor data profile
        return qos_profile_sensor_data

    def pipe_callback(self, msg, pipe):
        if self.livestream_enabled:
            self.pipe_publishers[pipe].publish(msg)

    def goal_callback(self, goal: RecordBag.Goal):
        self.get_logger().info(f"Received goal: {goal.name} {goal.max_duration}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f"Goal {goal_handle.goal_id} canceled")
        return CancelResponse.ACCEPT

    def record_bag_callback(self, goal_handle):
        record_bag_goal = goal_handle.request

        bag_name = record_bag_goal.name + '-' + datetime.datetime.now().strftime('%y_%m_%d-%H_%M_%S')
        bag_path = os.path.join(self.bag_dir_path, bag_name)

        self.get_logger().info(f"Recording bag '{bag_path}' for {record_bag_goal.max_duration} seconds")

        writer_choices = get_registered_writers()
        default_writer = 'sqlite3' if 'sqlite3' in writer_choices else writer_choices[0]
        storage_options = StorageOptions(
            uri=bag_path,
            storage_id=default_writer,
        )
        record_options = RecordOptions()
        record_options.topics = self.topics
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
        while rclpy.ok() and not goal_handle.is_cancel_requested and self.get_clock().now() - start < max_duration:
            self.get_clock().sleep_for(Duration(seconds=0.1))
        recorder.cancel()
        record_thread.join()
        self.get_logger().info(f"Recording finished. Bag saved at: {bag_path}")

        with self.input_nodes_running_services_lock:
            self.recording_bag = False
        self.toggle_input_nodes_running()

        result = RecordBag.Result()
        result.bag_path = bag_path
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = BagtubeServer()
    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
