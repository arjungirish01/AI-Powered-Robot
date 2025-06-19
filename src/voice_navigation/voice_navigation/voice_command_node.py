import os
import queue
import struct
import threading
import json
import yaml
import sounddevice as sd

from vosk import Model, KaldiRecognizer
import pvporcupine

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from transformers import pipeline
from TTS.api import TTS

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.get_logger().info("Voice command node started.")

        # --- Wake word ---
        access_key = os.getenv("PORCUPINE_ACCESS_KEY")
        if not access_key:
            self.get_logger().error("PORCUPINE_ACCESS_KEY not set.")
            exit(1)
        self.porcupine = pvporcupine.create(access_key=access_key, keywords=["porcupine"])
        self.frame_length = self.porcupine.frame_length

        # --- Vosk Speech Recognition ---
        model_path = os.getenv("VOSK_MODEL_PATH", os.path.expanduser("~/vosk-model-small-en-us-0.15"))
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Vosk model not found at: {model_path}")
            exit(1)
        self.model = Model(model_path)

        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'keypoints.yaml')
        with open(config_path, 'r') as f:
            self.keypoints = yaml.safe_load(f)

        grammar = list(self.keypoints.keys()) + ["go to", "navigate to"]
        self.recognizer = KaldiRecognizer(self.model, 16000, json.dumps(grammar))

        # --- ROS2 ---
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # --- TTS using Coqui ---
        self.tts_model = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=False, gpu=False)

        # --- General QA model ---
        self.qa_model = pipeline("question-answering")

        # --- Audio setup ---
        self.q = queue.Queue()
        self.stream = sd.InputStream(
            channels=1,
            samplerate=16000,
            dtype='int16',
            blocksize=self.frame_length,
            callback=self.audio_callback
        )
        self.stream.start()

        self.should_listen = True
        threading.Thread(target=self.main_loop, daemon=True).start()
        self.get_logger().info("Started main audio loop.")

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")
        self.q.put(bytes(indata))

    def main_loop(self):
        while rclpy.ok():
            if self.should_listen:
                pcm = self.q.get()
                unpacked = struct.unpack_from("h" * (len(pcm) // 2), pcm)
                if self.porcupine.process(unpacked) >= 0:
                    self.get_logger().info("Wake word detected!")
                    self.speak("Yes?")
                    self.handle_command()

    def handle_command(self):
        self.should_listen = False
        self.get_logger().info("Listening for command...")
        timeout = 5
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + timeout

        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            try:
                data = self.q.get(timeout=0.5)
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    text = result.get("text", "").lower()
                    if text:
                        self.get_logger().info(f"Heard: {text}")
                        self.process_command(text)
                        break
            except queue.Empty:
                continue
        self.should_listen = True

    def process_command(self, text):
        if text.startswith("navigate to") or text.startswith("go to"):
            self.handle_navigation_command(text)
        else:
            self.handle_general_query(text)

    def handle_navigation_command(self, text):
        text = text.replace("navigate to", "").replace("go to", "").strip()
        matched_key = None
        for key in self.keypoints:
            if key.replace("_", " ") in text:
                matched_key = key
                break

        if matched_key:
            self.get_logger().info(f"Navigating to: {matched_key}")
            self.speak(f"Navigating to {matched_key}.")
            self.send_goal(matched_key)
        else:
            self.get_logger().warn(f"No matching location found for: {text}")
            self.speak(f"I couldn't find the location {text}.")

    def send_goal(self, key):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.keypoints[key]['position']
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = self.keypoints[key]['orientation']
        self.publisher.publish(pose)

    def handle_general_query(self, text):
        self.get_logger().info(f"Answering general query: {text}")
        context = (
            "I am a robot assistant capable of navigating to locations and answering your questions. "
            "The restaurant and cafe are on the ground floor. "
            "The doctor's chamber is on the second floor. "
            "Reception is on the first floor. "
            "Smoking is strictly prohibited. "
            "My name is Porcupine. "
            "The telephone number is +49 171171171."
        )
        result = self.qa_model(question=text, context=context)
        answer = result['answer']
        self.get_logger().info(f"Answer: {answer}")
        self.speak(answer)

    def speak(self, text):
        self.get_logger().info(f"Speaking: {text}")
        def run_tts():
            wav = self.tts_model.tts(text)
            sd.play(wav, samplerate=22050)
            sd.wait()
        threading.Thread(target=run_tts, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

