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

    def shutdown_plugin(self):
        self._widget.close()
