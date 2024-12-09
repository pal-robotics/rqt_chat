import os
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
        _, package_path = get_resource('packages', 'rqt_chat')
        ui_file = os.path.join(package_path, 'share',
                               'rqt_chat', 'resource', 'chat.ui')
        loadUi(ui_file, self)

        # Fonts and Icons
        self.font = QFont()
        self.font.setPointSize(12)
        self.sendIcon = QIcon(os.path.join(
            package_path, 'share', 'rqt_chat', 'resource', 'send-circle.svg'))
        self.sendBtn.setIcon(self.sendIcon)

        self.userColor = QColor(15, 73, 44)
        self.userIcon = QIcon(os.path.join(
            package_path, 'share', 'rqt_chat', 'resource', 'face.svg'))
        self.robotColor = QColor(153, 60, 53)
        self.robotIcon = QIcon(os.path.join(
            package_path, 'share', 'rqt_chat', 'resource', 'robot.svg'))

        self.bgColor = QColor(240, 240, 240)

        self.intentColor = QColor(214, 233, 220)
        self.intentIcon = QIcon(os.path.join(
            package_path, 'share', 'rqt_chat', 'resource', 'intent.svg'))
        self.intentBgColor = QColor(98, 164, 132)

        # Grey for processing messages
        self.processingColor = QColor(100, 100, 100)
        self.processingIcon = QIcon(os.path.join(
            package_path, 'share', 'rqt_chat', 'resource', 'waiting-icon.svg'))

        # connect the sendBtn button to the send method
        self.sendBtn.clicked.connect(self.on_send)
        self.userInput.returnPressed.connect(self.on_send)
        self.msg_received.connect(self.msgHistory.addItem)

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
        if icon:
            item.setIcon(icon)
        return item

    def intent_input(self, msg):
        item = QListWidgetItem()
        text = f"Intent: [{msg.intent}]"

        if msg.data:
            text += f"\nData: {msg.data}"

        item.setText(text)
        item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        item.setForeground(self.intentColor)
        item.setBackground(self.intentBgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.intentIcon)
        return item

    def on_intent(self, msg):
        if self.showIntentsCheckbox.isChecked():
            self.msgQueue.put(self.intent_input(msg))

    def processing_message(self):
        item = QListWidgetItem("Processing...")
        item.setTextAlignment(Qt.AlignLeft)
        item.setForeground(self.processingColor)
        item.setBackground(self.bgColor)
        item.setFlags(Qt.NoItemFlags)
        item.setFont(self.font)
        item.setIcon(self.processingIcon)
        return item

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
        self.msgHistory.addItem(self.user_input(msg))

        # Publish user input
        live_speech = LiveSpeech()
        live_speech.final = msg
        self.speaker_list_pub.publish(IdsList(ids=[SPEAKER_NAME]))
        time.sleep(0.5)
        self.speech_pub.publish(live_speech)

        # Add "Processing..." message
        self.msgQueue.put(self.processing_message())

    def _say_cb(self, goal_handle):
        # Remove "Processing..." message when the robot starts responding
        if self.msgHistory.count() > 0:  # Ensure there are items to remove
            self.msgHistory.takeItem(self.msgHistory.count() - 1)

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
