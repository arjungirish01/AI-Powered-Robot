from rclpy.node import Node
class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        model_path = os.path.expanduser("~/vosk-model-small-en-us-0.15")
        self.get_logger().info(f"Checking Vosk model path: {model_path}")
        exists = os.path.exists(model_path)
        self.get_logger().info(f"Does model path exist? {exists}")

        if not exists:
            self.get_logger().error(f"Vosk model not found at: {model_path}")
            raise FileNotFoundError(f"Vosk model not found at: {model_path}")

        # The rest of your initialization code here

