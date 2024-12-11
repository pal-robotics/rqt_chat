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
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QListWidgetItem
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from hri_msgs.msg import LiveSpeech, IdsList
from hri_actions_msgs.msg import Intent
from tts_msgs.action import TTS

SPEAKER_NAME = "anonymous_speaker"
SPEECH_TOPIC = f"/humans/voices/{SPEAKER_NAME}/speech"

PKG_PATH = Path(get_resource('packages', 'rqt_chat')[1])
RESOURCE_PATH = PKG_PATH / 'share' / 'rqt_chat' / 'resource'

class IntentItem(QListWidgetItem):

    IntentType = QListWidgetItem.UserType + 1

    def __init__(self, msg):
        super(IntentItem, self).__init__(type=self.IntentType)

        text = f"Intent: [{msg.intent}]"

        if msg.data:
            text += f"\nData: {msg.data}"

        self.font = QFont()
        self.font.setPointSize(12)
        self.setText(text)
        self.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.setForeground(QColor(114, 133, 120))
        self.setFlags(Qt.NoItemFlags)
        self.setFont(self.font)
        self.setIcon(QIcon(str(RESOURCE_PATH / 'intent.svg')))

class ProcessingSpinnerItem(QListWidgetItem):

    ProcessingSpinnerType = QListWidgetItem.UserType + 2

    def __init__(self):
        super(ProcessingSpinnerItem, self).__init__(type=self.ProcessingSpinnerType)

        self.font = QFont()
        self.font.setPointSize(12)

        self.processingIcon = QIcon(str(RESOURCE_PATH / 'waiting-icon.svg'))

        self.setText("Processing...")
        self.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.setForeground(QColor(100, 100, 100))
        self.setFlags(Qt.NoItemFlags)
        self.setFont(self.font)
        self.setIcon(self.processingIcon)

class ChatWidget(QWidget):
    # Signal for thread-safe message addition
    msg_received = pyqtSignal(QListWidgetItem)

    def __init__(self, node, plugin):
        super(ChatWidget, self).__init__()

        self._node = node
        self._plugin = plugin
        self.msgQueue = queue.Queue()
        self.update_thread_running = True

        # Load UI file
        ui_file  = RESOURCE_PATH / 'chat.ui'
        loadUi(str(ui_file), self)

        self.msgHistory.setStyleSheet("QListWidget:item { margin-bottom: 10px; background-color: #f7f7f7; }")


        # Fonts and Icons
        self.font = QFont()
        self.font.setPointSize(12)
        self.sendIcon = QIcon(str(RESOURCE_PATH / 'send-circle.svg'))
        self.sendBtn.setIcon(self.sendIcon)

        self.userColor = QColor(15, 73, 44)
        self.userIcon = QIcon(str(RESOURCE_PATH / 'face.svg'))
        self.robotColor = QColor(153, 60, 53)
        self.robotIcon = QIcon(str(RESOURCE_PATH / 'robot.svg'))

        self.bgColor = QColor(240, 240, 240)

        # connect the sendBtn button to the send method
        self.sendBtn.clicked.connect(self.on_send)
        self.userInput.returnPressed.connect(self.on_send)
        self.msg_received.connect(self.add_item)

        # connect the showIntentsCheckbox to the toggle_intents method
        self.showIntentsCheckbox.stateChanged.connect(self.toggle_intents)

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

        self._node.get_logger().info("loaded")

    def user_input(self, msg):

        item = QListWidgetItem(msg)
        item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        item.setForeground(self.userColor)
        item.setBackground(self.bgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.userIcon)

        return item

    def robot_input(self, msg, icon=None):
        item = QListWidgetItem(msg)
        item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        item.setForeground(self.robotColor)
        item.setBackground(self.bgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.robotIcon)

        return item

    def on_intent(self, msg):
        self.msgQueue.put(IntentItem(msg))

    def toggle_intents(self, state):
        for i in range(self.msgHistory.count()):
            item = self.msgHistory.item(i)
            if item.type() == IntentItem.IntentType:
                item.setHidden(not state)

    def add_item(self, item):
        self.msgHistory.addItem(item)
        if item.type() == IntentItem.IntentType \
           and not self.showIntentsCheckbox.isChecked():
               item.setHidden(True)

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
        self.add_item(self.user_input(msg))

        # Publish user input
        live_speech = LiveSpeech()
        live_speech.final = msg
        self.speaker_list_pub.publish(IdsList(ids=[SPEAKER_NAME]))
        time.sleep(0.5)
        self.speech_pub.publish(live_speech)

        # Add "Processing..." message
        self.add_item(ProcessingSpinnerItem())

    def _say_cb(self, goal_handle):

        # Find and remove the "Processing..." message if any
        indices = []
        for i in range(self.msgHistory.count()):
            item = self.msgHistory.item(i)
            if item.type() == ProcessingSpinnerItem.ProcessingSpinnerType:
                indices.append(i)
        for i in indices:
            self.msgHistory.takeItem(i)

        # Process robot response
        txt = goal_handle.request.input
        self.msgQueue.put(self.robot_input(txt))

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
