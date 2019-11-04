import os
import rospy
from std_msgs import msg
# import rospkg


from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QAbstractButton

class MovementWidget(QWidget):

    def __init__(self, context):
        super(MovementWidget, self).__init__()

        # Register publishers
        self.publishers = {}
        self.register_publishers()
        
        # Extend the widget with all attributes and children from UI file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'GUI.ui')
        loadUi(ui_file, self)
        self.setObjectName('MovementWidget')


        # Register click handlers
        self.movement_button_group.buttonClicked[QAbstractButton].connect(self._handle_movement_btns)
        self.calibrate_btn.clicked.connect(self._handle_calibrate_btn_click)

        # Register the keypress event handlers. 
        self.keyPressEvent = self.on_key_press
        
        # Initialise policies and variables needed to trigger focus events
        self.setFocusPolicy(Qt.StrongFocus)
        self.current_focus = ""

    

    def _handle_movement_btns(self,button):
        btn_text = button.text()
        self._send_movement_message(btn_text)        

    def _send_movement_message(self,btn_text):

        message = msg.Float32MultiArray()

        if (btn_text == "Forward"):
            self.create_alert('You clicked the {0} button!'.format(btn_text.lower()))
            message.data = [1,1]
        elif (btn_text == "Backward"):
            self.create_alert('You clicked the {0} button!'.format(btn_text.lower()))
            message.data = [-1,-1]
        elif (btn_text == "Left"):
            self.create_alert('You clicked the {0} button!'.format(btn_text.lower()))
            message.data = [0,1]
        else:
            self.create_alert('You clicked the {0} button!'.format(btn_text.lower()))
            message.data = [1,0]

        self.publishers['drive/distance'].publish(message)

    def _handle_calibrate_btn_click(self):
        self.publishers['drive/commands'].publish(ord('r'))   # We need to make sure that there is a way for the driver to callback and say that calibration is done!


    @staticmethod
    def create_alert(alert_msg):
        alert = QMessageBox()
        alert.setText(alert_msg)
        alert.exec_()

    # callback for ui events
    def on_key_press(self,event):
        key = event.key()
        if key == Qt.Key_Up:
            self._send_movement_message("Forward")
        elif key == Qt.Key_Down:
            self._send_movement_message("Backward")
        elif key == Qt.Key_Left:
            self._send_movement_message("Left")
        elif key == Qt.Key_Right:
            self._send_movement_message("Right")


    # These 2 events are used to grab / release the keyboard as the focus on the widget changes.
    # This is important to prevent Qt from grabbing control of the arrow keys to shift the current button focus
    # self.current_focus is a selector used to handle the large volume of focusInEvents. 
    def focusInEvent(self,e):
        if (self.current_focus != "in"):
            # self.create_alert("focus in")
            self.grabKeyboard()
            self.current_focus = "in"
        super(MovementWidget, self).focusInEvent(e)
    
    def focusOutEvent(self,e):
        if (self.current_focus != "out"):
            # self.create_alert("focus out")
            self.releaseKeyboard()
            self.current_focus = "out"
        super(MovementWidget, self).focusOutEvent(e)

    def register_publishers(self):
        self.publishers = {
            'drive/commands': rospy.Publisher('drive/commands', msg.Char, queue_size=10),
            'drive/distance': rospy.Publisher('drive/distance', msg.Float32MultiArray, queue_size=10)
        }

    def unregister_publishers(self):
        for publisher in self.publishers:
            publisher.unregister()