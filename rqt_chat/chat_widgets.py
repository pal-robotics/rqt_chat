from PyQt5.QtWidgets import QApplication, QWidget, QListWidget, QListWidgetItem, QVBoxLayout, QLabel, QHBoxLayout, QSizePolicy
from PyQt5.QtGui import QPainter, QColor, QPainterPath, QMovie, QIcon, QFont
from PyQt5.QtCore import Qt, QRectF, QSize, QTimer, pyqtSignal
import sys
from pathlib import Path


class ContentWidget(QWidget):
    """The inner content widget with rounded corners and text"""

    def __init__(self, text, bg_color="#ffffff",  parent=None):
        super().__init__(parent)
        self.text = text
        self.bg_color = bg_color
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)

        # Add label with word wrapping
        self.label = QLabel(self.text)
        self.label.setStyleSheet(
            "color: white; background-color: transparent;")
        self.label.setWordWrap(True)
        self.label.setTextFormat(Qt.TextFormat.RichText)
        self.label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        font = QFont()
        font.setPointSize(10)
        self.label.setFont(font)

        layout.addWidget(self.label)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        path = QPainterPath()
        path.addRoundedRect(QRectF(0, 0, self.width(), self.height()), 10, 10)
        painter.fillPath(path, QColor(self.bg_color))


class BubbleWidget(QWidget):
    """Container widget that handles alignment and sizing"""

    def __init__(self, text, align_right=False, bg_color="#ffffff", width_percent=70, parent=None):
        super().__init__(parent)
        self.text = text
        self.align_right = align_right
        self.width_percent = width_percent

        # Create layout
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        # Create content widget
        self.content = ContentWidget(text, bg_color)

        # Set up alignment
        self.updateLayout()

    def updateLayout(self):
        """Set up or update the layout based on alignment"""
        # Clear existing layout
        while self.layout.count():
            item = self.layout.takeAt(0)
            if item.widget() is not self.content:
                del item

        # Add content with proper alignment
        if self.align_right:
            self.layout.addStretch(1)
            self.layout.addWidget(self.content)
        else:
            self.layout.addWidget(self.content)
            self.layout.addStretch(1)

    def updateContentWidth(self):
        """Update content width based on parent width"""
        width = int(self.width() * (self.width_percent / 100))
        self.content.setFixedWidth(width)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.updateContentWidth()


class IntentWidget(QWidget):

    def __init__(self, msg, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TranslucentBackground)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        text = f"Intent: [{msg.intent}]"

        if msg.data:
            text += f"\nData: {msg.data}"

        label = QLabel(text)
        # use a monospaced font
        font = QFont()
        font.setFamily("monospace")
        font.setPointSize(10)
        label.setFont(font)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        icon = QIcon(str(ChatListWidget.resource_path / 'intent.svg'))

        icon_label = QLabel()
        icon_label.setPixmap(icon.pixmap(24, 24))
        icon_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        layout.addWidget(icon_label)
        layout.addWidget(label, 1)
        self.setLayout(layout)

    def updateContentWidth(self):
        pass


class ProcessingWidget(QWidget):
    """Widget showing a loading GIF that auto-destructs"""

    remove_requested = pyqtSignal(object)

    def __init__(self, parent=None, duration=5000):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.duration = duration

        # Set up layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setAlignment(Qt.AlignCenter)

        # Create spinner with animated GIF
        self.spinner = QLabel()
        self.spinner.setAlignment(Qt.AlignCenter)
        self.spinner.setStyleSheet("background-color: transparent;")

        self.movie = QMovie(str(ChatListWidget.resource_path / 'spinner.gif'))
        self.movie.setScaledSize(QSize(60, 40))  # Adjust size as needed
        self.spinner.setMovie(self.movie)
        self.movie.start()

        layout.addWidget(self.spinner)

        # Set up auto-destruct timer
        self.destruct_timer = QTimer(self)
        self.destruct_timer.timeout.connect(self.remove_from_list)
        self.destruct_timer.setSingleShot(True)
        self.destruct_timer.start(self.duration)

    def remove_from_list(self):
        """Remove this widget from its parent list widget"""
        self.movie.stop()
        self.remove_requested.emit(self)

    def updateContentWidth(self):
        pass


class ChatListWidget(QListWidget):
    """Custom list widget optimized for chat-like display"""

    resource_path = Path(__file__).parent / 'resource'

    def __init__(self, resource_path=None, parent=None):
        super().__init__(parent)

        if resource_path:
            ChatListWidget.resource_path = resource_path

        self.setSpacing(5)
        self.setStyleSheet("""
            QListWidget {
                background-color: white;
                outline: none;
                border: none;
            }
            QListWidget::item {
                background-color: transparent;
            }
            QListWidget::item:selected {
                background-color: transparent;
            }
        """)

        self.display_intents = False
        self.last_processing_item = None

        # Set up resize handling
        self.resizeTimer = QTimer(self)
        self.resizeTimer.setSingleShot(True)
        self.resizeTimer.timeout.connect(self.updateItemSizes)
        self.viewport().installEventFilter(self)

    def toggle_intents(self, visible: bool):
        for i in range(self.count()):
            item = self.item(i)
            widget = self.itemWidget(item)
            if widget and isinstance(widget, IntentWidget):
                item.setHidden(not visible)

    def addChatItem(self, text, align_right=False, bg_color="#3498db", icon=None):
        """Add a chat bubble to the list"""

        if self.last_processing_item:
            self.removeItemWidget(self.last_processing_item)

        # Create list item
        item = QListWidgetItem(self)
        bubble = BubbleWidget(text, align_right, bg_color)

        # Add to list
        item.setSizeHint(QSize(self.viewport().width(), 50))
        self.addItem(item)
        self.setItemWidget(item, bubble)

        # Update size after adding
        QTimer.singleShot(10, self.updateItemSizes)

        return item

    def addIntentItem(self, msg):
        """Add an intent widget to the list"""

        # Create list item
        item = QListWidgetItem(self)
        intent = IntentWidget(msg)

        # Add to list
        item.setSizeHint(QSize(self.viewport().width(), 50))
        item.setHidden(not self.display_intents)
        self.addItem(item)
        self.setItemWidget(item, intent)

        return item

    def addProcessingItem(self):
        """Add a processing widget to the list"""

        # Create list item
        item = QListWidgetItem(self)
        processing = ProcessingWidget()

        processing.remove_requested.connect(
            lambda item: self.removeItemWidget(item))

        self.last_processing_item = processing

        # Add to list
        item.setSizeHint(QSize(self.viewport().width(), 50))
        self.addItem(item)
        self.setItemWidget(item, processing)

        return item

    def removeItemWidget(self, item: QWidget):

        # find the QListWidgetItem for the QWidget
        # and remove it from the list
        for i in range(self.count()):
            if self.itemWidget(self.item(i)) == item:
                self.takeItem(i)
                break

    def updateItemSizes(self):
        """Update all items sizes based on content"""
        width = self.viewport().width()
        for i in range(self.count()):
            item = self.item(i)
            widget = self.itemWidget(item)
            if widget:
                widget.updateContentWidth()
                item.setSizeHint(QSize(width, widget.sizeHint().height()))

    def eventFilter(self, obj, event):
        """Watch for resize events on the viewport"""
        if obj is self.viewport() and event.type() == event.Resize:
            self.resizeTimer.start(50)
        return super().eventFilter(obj, event)

    def resizeEvent(self, event):
        """Handle list widget resize events"""
        super().resizeEvent(event)
        self.resizeTimer.start(50)


if __name__ == '__main__':

    class Intent:
        def __init__(self, intent, data=None):
            self.intent = intent
            self.data = data

    class MainWindow(QWidget):
        def __init__(self):
            super().__init__()
            self.setGeometry(300, 300, 400, 400)
            self.setWindowTitle('Chat Bubbles Example')

            # Create layout
            layout = QVBoxLayout(self)

            # Create chat list
            self.chatList = ChatListWidget()
            layout.addWidget(self.chatList)

            # Add some test messages
            messages = [
                "Short text",
                "Medium length text that might wrap depending on the width",
                "This is a much longer text that will definitely need to wrap to multiple lines when displayed in our chat bubble. It's designed to demonstrate how text wrapping works in our custom widget.",
                "Another short one",
                "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nullam auctor, nisl eget ultricies tincidunt, nunc nisl aliquet nunc, quis aliquam nisl nunc quis nisl. Donec euismod, nisl eget ultricies tincidunt."
            ]

            for i, msg in enumerate(messages):
                self.chatList.addChatItem(msg, align_right=(
                    i % 2 == 1), bg_color="#3498db" if i % 2 == 0 else "#e74c3c")
                self.chatList.addProcessingItem()

            self.chatList.addIntentItem(Intent("greet"))

    app = QApplication(sys.argv)
    ex = MainWindow()
    ex.show()
    sys.exit(app.exec_())
