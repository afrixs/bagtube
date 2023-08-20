import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class BagtubeServer(Node):
    def __init__(self):
        super().__init__('bagtube_server')
        pipes = self.declare_parameter('pipes', rclpy.Parameter.Type.STRING_ARRAY).get_parameter_value().string_array_value

        self.pipe_publishers = {}
        self.pipe_subscribers = {}

        for pipe in pipes:
            type_str = self.declare_parameter(f'{pipe}.type', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            input_topic = self.declare_parameter(f'{pipe}.input_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            output_topic = self.declare_parameter(f'{pipe}.output_topic', rclpy.Parameter.Type.STRING).get_parameter_value().string_value

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

    def load_message_type(self, type_str):
        try:
            module_name, class_name = type_str.split('/')
            module = __import__(f'{module_name}.msg', fromlist=[class_name])
            msg_type = getattr(module, class_name)
            return msg_type
        except (ValueError, ImportError, AttributeError):
            return None

    def get_qos_profile_for_msg_type(self, msg_type):
        # Define custom QoS profiles for specific message types if needed
        # For now, using a sensor data profile
        return qos_profile_sensor_data

    def pipe_callback(self, msg, pipe):
        # self.get_logger().info(f'Received: {msg} on pipe {pipe}')
        self.pipe_publishers[pipe].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BagtubeServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()