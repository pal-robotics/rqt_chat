import os
from ament_index_python import get_resource
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtWidgets import QWidget, QListWidgetItem

from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from hri_msgs.msg import LiveSpeech, IdsList
from tts_msgs.action import TTS

SPEAKER_NAME = "anonymous_speaker"
SPEECH_TOPIC = f"/humans/voices/{SPEAKER_NAME}/speech"

class ChatWidget(QWidget):
    def __init__(self, node, plugin):
        super(ChatWidget, self).__init__()

        self._node = node
        self._plugin = plugin

        _, package_path = get_resource('packages', 'rqt_chat')
        ui_file = os.path.join(package_path, 'share', 'rqt_chat', 'resource', 'chat.ui')
        loadUi(ui_file, self)

        # connect the sendBtn button to the send method
        self.sendBtn.clicked.connect(self.on_send)
        self.userInput.returnPressed.connect(self.on_send)

        self.msgHistory.setSpacing(3)
        self.msgHistory.addItem(self.robot_input("[you can start chatting with the robot here]"))

        # publish the list of tracked 'voices'
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.speaker_list_pub = self._node.create_publisher(IdsList, "/humans/voices/tracked", latching_qos)
        self.speaker_list_pub.publish(IdsList(ids=[SPEAKER_NAME]))

        self.speech_pub = self._node.create_publisher(LiveSpeech, SPEECH_TOPIC, 10)


        # create a ROS action server for the '/say' action (type: tts_msgs/action/TTS)
        self._action_server = ActionServer(self._node, TTS, '/tts_engine/tts', self._say_cb)

        self._node.get_logger().info("loaded")

    def user_input(self, msg):

        item = QListWidgetItem(msg)
        item.setTextAlignment(QtCore.Qt.AlignRight)
        item.setForeground(QtCore.Qt.blue)
        item.setBackground(QtCore.Qt.lightGray)
        item.setFlags(QtCore.Qt.NoItemFlags)

        return item

    def robot_input(self, msg):

        item = QListWidgetItem(msg)
        item.setTextAlignment(QtCore.Qt.AlignLeft)
        item.setForeground(QtCore.Qt.red)
        item.setBackground(QtCore.Qt.lightGray)
        item.setFlags(QtCore.Qt.NoItemFlags)

        return item

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
        # add the msg to the msgHistory as a robot input
        self.msgHistory.addItem(self.robot_input(goal_handle.request.input))
        goal_handle.succeed()


