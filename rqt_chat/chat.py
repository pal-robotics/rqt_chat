from qt_gui.plugin import Plugin

from .chat_widget import ChatWidget


class ChatPlugin(Plugin):
    def __init__(self, context):
        super(ChatPlugin, self).__init__(context)

        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQtChat')

        self._widget = ChatWidget(self._node, self)
        context.add_widget(self._widget)
