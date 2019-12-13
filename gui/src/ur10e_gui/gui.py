import os
import rospy
import rospkg
from enum import Enum

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtCore import Slot, pyqtSignal
from python_qt_binding.QtWidgets import QWidget

from std_msgs.msg import Empty
from std_msgs.msg import UInt16


class State:
    standstill = 1
    cart_velocity = 10
    joint_position = 20
    stopping = 100
    error = 200


class Gui(Plugin):
    state_updated = pyqtSignal()

    def __init__(self, context):
        super(Gui, self).__init__(context)
        self.setObjectName('Gui')

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'ur10e_gui'), 'resource', 'gui.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('GuiUi')
        context.add_widget(self._widget)

        self.state = 0

        self._widget.pb_stand.clicked.connect(self.stand_pressed)
        self._widget.pb_velocity.clicked.connect(self.velocity_pressed)
        self._widget.pb_position.clicked.connect(self.position_pressed)
        self._widget.pb_stop.clicked.connect(self.stop_pressed)
        self._widget.pb_back_vel.clicked.connect(self.back)
        self._widget.pb_back_pos.clicked.connect(self.back)
        self._widget.pb_back_error.clicked.connect(self.back)

        self.state_updated.connect(self.update_state)

        self.reset_pub = rospy.Publisher('ur10e/reset', Empty, queue_size=1)
        self.stop_pub = rospy.Publisher('ur10e/stop', Empty, queue_size=1)
        self.start_vel_pub = rospy.Publisher('ur10e/start_vel', Empty, queue_size=1)
        self.start_pos_pub = rospy.Publisher('ur10e/start_pos', Empty, queue_size=1)

        rospy.Subscriber("ur10e/state", UInt16, self.ros_update_state)

        self.update_state()

    @Slot()
    def back(self):
        self._widget.tab.setCurrentIndex(0)

    @Slot()
    def stand_pressed(self):
        if self.state == State.error:
            self.reset_pub.publish()

    @Slot()
    def velocity_pressed(self):
        if self.state == State.cart_velocity:
            self._widget.tab.setCurrentIndex(1)
        self.start_vel_pub.publish()
        pass

    @Slot()
    def position_pressed(self):
        self.start_pos_pub.publish()
        pass

    @Slot()
    def stop_pressed(self):
        self.stop_pub.publish()
        pass

    @Slot()
    def update_state(self):
        # disable all buttons
        self._widget.pb_stand.setEnabled(False)
        self._widget.pb_stand.setStyleSheet("background-color: white")
        self._widget.pb_position.setEnabled(False)
        self._widget.pb_position.setStyleSheet("background-color: white")
        self._widget.pb_velocity.setEnabled(False)
        self._widget.pb_velocity.setStyleSheet("background-color: white")
        self._widget.pb_stop.setEnabled(False)
        self._widget.pb_stop.setStyleSheet("background-color: white")
        self._widget.pb_error.setEnabled(False)
        self._widget.pb_error.setStyleSheet("background-color: white")

        rospy.loginfo("state: %d", self.state)

        if self.state == State.standstill:
            self._widget.pb_position.setEnabled(True)
            self._widget.pb_stand.setEnabled(True)
            self._widget.pb_velocity.setEnabled(True)
            self._widget.pb_stand.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.cart_velocity:
            self._widget.pb_velocity.setEnabled(True)
            self._widget.pb_stop.setEnabled(True)
            self._widget.pb_velocity.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.joint_position:
            self._widget.pb_position.setEnabled(True)
            self._widget.pb_stop.setEnabled(True)
            self._widget.pb_position.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.stopping:
            self._widget.pb_stop.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.error:
            self._widget.pb_error.setEnabled(True)
            self._widget.pb_error.setStyleSheet("background-color:rgb(245, 121, 0)")
        pass

    def ros_update_state(self, state):
        if state.data != self.state:
            self.state = state.data
            self.state_updated.emit()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # todo: save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # todo: restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
