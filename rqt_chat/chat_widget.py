import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QFont, QColor, QIcon
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QListWidgetItem

from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from hri_msgs.msg import LiveSpeech, IdsList
from tts_msgs.action import TTS

import time

import queue
import threading

SPEAKER_NAME = "anonymous_speaker"
SPEECH_TOPIC = f"/humans/voices/{SPEAKER_NAME}/speech"


class ChatWidget(QWidget):
    def __init__(self, node, plugin):
        super(ChatWidget, self).__init__()

        self._node = node
        self._plugin = plugin

        self.msgQueue = queue.Queue()

        _, package_path = get_resource('packages', 'rqt_chat')
        ui_file = os.path.join(package_path, 'share', 'rqt_chat', 'resource', 'chat.ui')
        loadUi(ui_file, self)

        self.font = QFont()
        self.font.setPointSize(12)

        self.sendIcon = QIcon(os.path.join(package_path,
                                           'share',
                                           'rqt_chat',
                                           'resource',
                                           'send-circle.svg'))
        self.sendBtn.setIcon(self.sendIcon)

        self.userColor = QColor(15, 73, 44)
        self.userIcon = QIcon(os.path.join(package_path,
                                           'share',
                                           'rqt_chat',
                                           'resource',
                                           'face.svg'))
        self.robotColor = QColor(153, 60, 53)
        self.robotIcon = QIcon(os.path.join(package_path,
                                            'share',
                                            'rqt_chat',
                                            'resource',
                                            'robot.svg'))
        self.bgColor = QColor(240, 240, 240)

        # connect the sendBtn button to the send method
        self.sendBtn.clicked.connect(self.on_send)
        self.userInput.returnPressed.connect(self.on_send)

        # publish the list of tracked 'voices'
        latching_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.speaker_list_pub = self._node.create_publisher(IdsList,
                                                            "/humans/voices/tracked",
                                                            latching_qos)
        self.speaker_list_pub.publish(IdsList(ids=[SPEAKER_NAME]))

        self.speech_pub = self._node.create_publisher(LiveSpeech, SPEECH_TOPIC, 10)

        # create a ROS action server for the '/say' action (type: tts_msgs/action/TTS)
        self._action_server = ActionServer(self._node, TTS, '/tts_engine/tts', self._say_cb)

        # start a thread to update the msgHistory list
        self.update_thread = threading.Thread(target=self.update_msg_list)
        self.update_thread.start()

        self._node.get_logger().info("loaded")

    def user_input(self, msg):

        item = QListWidgetItem(msg)
        item.setTextAlignment(Qt.AlignRight)
        item.setForeground(self.userColor)
        item.setBackground(self.bgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.userIcon)

        return item

    def robot_input(self, msg):

        item = QListWidgetItem(msg)
        item.setTextAlignment(Qt.AlignLeft)
        item.setForeground(self.robotColor)
        item.setBackground(self.bgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.robotIcon)

        return item

    def update_msg_list(self):
        while True:
            msg = self.msgQueue.get()
            self.msgHistory.addItem(msg)

    def on_send(self, flags=None):
        # get the text from the input field, and add it to the QListWidget model
        msg = self.userInput.text()
        self.userInput.clear()
        self.userInput.setPlaceholderText("")
        self.userInput.setFocus()

        live_speech = LiveSpeech()
        live_speech.final = msg

        self.msgHistory.addItem(self.user_input(msg))
        self.speech_pub.publish(live_speech)

    def _say_cb(self, goal_handle):

        txt = goal_handle.request.input
        self.msgQueue.put(self.robot_input(txt))

        for word in txt.split():
            feedback = TTS.Feedback()
            feedback.word = word
            goal_handle.publish_feedback(feedback)
            self._node.get_logger().info(f"Robot saying <{word}>...")
            time.sleep(0.3)

        self._node.get_logger().info("Robot done speaking!")

        goal_handle.succeed()
        result = TTS.Result()
        result.error_msg = ""
        return result
