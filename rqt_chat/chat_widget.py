# Copyright 2024 pal-robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import queue
import threading
import time
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QFont, QColor, QIcon
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from hri_msgs.msg import LiveSpeech, IdsList
from hri_actions_msgs.msg import Intent
from tts_msgs.action import TTS

from .chat_widgets import ChatListWidget

SPEAKER_NAME = "anonymous_speaker"
SPEECH_TOPIC = f"/humans/voices/{SPEAKER_NAME}/speech"

PKG_PATH = Path(get_resource('packages', 'rqt_chat')[1])
RESOURCE_PATH = PKG_PATH / 'share' / 'rqt_chat' / 'resource'


class AsyncMsg:

    def __init__(self, type=None, data=None):
        self.type = type
        self.data = data


class ChatWidget(QWidget):
    # Signal for thread-safe message addition
    msg_received = pyqtSignal(AsyncMsg)

    def __init__(self, node, plugin):
        super(ChatWidget, self).__init__()

        self._node = node
        self._plugin = plugin
        self.msgQueue = queue.Queue()
        self.update_thread_running = True

        # Load UI file
        ui_file = RESOURCE_PATH / 'chat.ui'
        loadUi(str(ui_file), self)

        self.msgHistory = ChatListWidget(RESOURCE_PATH)
        self.verticalLayout.insertWidget(0, self.msgHistory)

        # Fonts and Icons
        self.font = QFont()
        self.font.setPointSize(12)
        self.sendIcon = QIcon(str(RESOURCE_PATH / 'send-circle.svg'))
        self.sendBtn.setIcon(self.sendIcon)

        self.user_color = "#3498db"
        self.userIcon = QIcon(str(RESOURCE_PATH / 'face.svg'))
        self.robot_color = "#373737"
        self.robotIcon = QIcon(str(RESOURCE_PATH / 'robot.svg'))

        self.bgColor = QColor(240, 240, 240)

        # connect the sendBtn button to the send method
        self.sendBtn.clicked.connect(self.on_send)
        self.userInput.returnPressed.connect(self.on_send)
        self.msg_received.connect(self.add_remote_msg)

        # connect the showIntentsCheckbox to the toggle_intents method
        self.showIntentsCheckbox.stateChanged.connect(
            self.msgHistory.toggle_intents)
        self.msgHistory.toggle_intents(self.showIntentsCheckbox.isChecked())

        # publish the list of tracked 'voices'
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.speaker_list_pub = self._node.create_publisher(
            IdsList, "/humans/voices/tracked", latching_qos)
        self.speech_pub = self._node.create_publisher(
            LiveSpeech, SPEECH_TOPIC, 10)

        self.intents_sub = self._node.create_subscription(
            Intent, "/intents",
            self.on_intent, 1)
        self._action_server = ActionServer(
            self._node, TTS, '/tts_engine/tts', self._say_cb)

        # create a ROS action server for the '/say' action (type: tts_msgs/action/TTS)
        self.update_thread = threading.Thread(target=self.update_msg_list)
        self.update_thread.start()

    def user_input(self, msg):
        self.msgHistory.addChatItem(
            msg, align_right=True, bg_color=self.user_color, icon=self.userIcon)

        self.msgHistory.scrollToBottom()

    def on_intent(self, msg):
        self.msgQueue.put(AsyncMsg(type="Intent", data=msg))

    def add_remote_msg(self, msg):
        if msg.type == "Intent":
            self.msgHistory.addIntentItem(msg.data)
        elif msg.type == "Robot":
            self.msgHistory.addChatItem(
                msg.data, align_right=False, bg_color=self.robot_color, icon=self.robotIcon)
        else:
            self._node.get_logger().warn(f"Unknown message type: {msg.type}")

        self.msgHistory.scrollToBottom()

    def update_msg_list(self):
        while self.update_thread_running:
            msg = self.msgQueue.get()
            self.msg_received.emit(msg)

    def on_send(self, flags=None):
        # Capture user input
        msg = self.userInput.text()
        self.userInput.clear()
        self.userInput.setPlaceholderText("")
        self.userInput.setFocus()

        # Add user message
        self.user_input(msg)

        # Publish user input
        live_speech = LiveSpeech()
        live_speech.final = msg
        self.speaker_list_pub.publish(IdsList(ids=[SPEAKER_NAME]))
        time.sleep(0.5)
        self.speech_pub.publish(live_speech)

        # Add "Processing..." message
        self.msgHistory.addProcessingItem()

    def _say_cb(self, goal_handle):

        # Process robot response
        txt = goal_handle.request.input
        self.msgQueue.put(AsyncMsg(type="Robot", data=txt))

        for word in txt.split():
            feedback = TTS.Feedback()
            feedback.word = word
            goal_handle.publish_feedback(feedback)
            self._node.get_logger().info(f"Robot saying <{word}>...")
            time.sleep(0.3)

        # Action success
        self._node.get_logger().info("Robot done speaking!")
        goal_handle.succeed()
        result = TTS.Result()
        result.error_msg = ""
        return result

    def closeEvent(self, event):
        # Clean up background thread
        self.update_thread_running = False
        self.update_thread.join()
        super(ChatWidget, self).closeEvent(event)
