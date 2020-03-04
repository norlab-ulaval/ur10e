import os
import rospy
import rospkg
import rosservice
from enum import Enum

from qt_gui.plugin import Plugin
from PyQt5.QtCore import Slot, pyqtSignal, QTimer
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from std_srvs.srv import Empty
from ur10e_messages.msg import State


# connect to services
stop_trajectory = rospy.ServiceProxy('ur10e/stop_traj', Empty)
start_velocity = rospy.ServiceProxy('ur10e/start_vel', Empty)
start_position = rospy.ServiceProxy('ur10e/start_pos', Empty)
reset_errors = rospy.ServiceProxy('ur10e/reset_errors', Empty)
init = rospy.ServiceProxy('ur10e/init', Empty)
home = rospy.ServiceProxy('ur10e/home', Empty)


class Gui(Plugin):
    sig_update_state = pyqtSignal()

    def __init__(self, context):
        super(Gui, self).__init__(context)
        self.setObjectName('Gui')
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('ur10e_gui'), 'resource', 'gui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('GuiUi')
        context.add_widget(self._widget)

        self.state = State.uninitialized
        self.update_state()

        # connect signals
        self._widget.pb_home.clicked.connect(self.home_pressed)
        self._widget.pb_start.clicked.connect(self.start_pressed)
        self._widget.pb_velocity.clicked.connect(self.velocity_pressed)
        self._widget.pb_position.clicked.connect(self.position_pressed)
        self._widget.pb_stop.clicked.connect(self.stop_pressed)
        self._widget.pb_error.clicked.connect(self.error_pressed)
        self._widget.pb_reset.clicked.connect(self.reset_pressed)
        self._widget.pb_back_vel.clicked.connect(self.back)
        self._widget.pb_back_pos.clicked.connect(self.back)
        self._widget.pb_back_home.clicked.connect(self.back)
        self._widget.pb_back_error.clicked.connect(self.back)
        self.sig_update_state.connect(self.update_state)

        # subscribe to topics
        self.state_sub = rospy.Subscriber('ur10e/state', State, self.ros_update_state)

    def ros_update_state(self, state):
        if state.state != self.state:
            self.state = state.state
            self.sig_update_state.emit()

    @Slot()
    def update_state(self):
        print(self.state)

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
        self._widget.pb_home.setEnabled(False)
        self._widget.pb_home.setStyleSheet("background-color: white")
        self._widget.pb_start.setEnabled(False)
        self._widget.pb_start.setStyleSheet("background-color: white")

        if self.state == State.uninitialized:
            self._widget.pb_start.setEnabled(True)
            self._widget.pb_start.setStyleSheet(
                "background-color:rgb(114, 159, 207); color: white;"
            )

        elif self.state == State.standstill:
            self._widget.pb_position.setEnabled(True)
            self._widget.pb_velocity.setEnabled(True)
            self._widget.pb_home.setEnabled(True)
            self._widget.pb_stand.setStyleSheet("background-color:rgb(114, 159, 207); color:white")

        elif self.state == State.cart_velocity:
            self._widget.pb_velocity.setEnabled(True)
            self._widget.pb_stop.setEnabled(True)
            self._widget.pb_velocity.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.homing:
            self._widget.pb_home.setEnabled(True)
            self._widget.pb_stop.setEnabled(True)
            self._widget.pb_home.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.joint_position:
            self._widget.pb_position.setEnabled(True)
            self._widget.pb_stop.setEnabled(True)
            self._widget.pb_position.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.stopping:
            self._widget.pb_stop.setStyleSheet("background-color:rgb(114, 159, 207)")

        elif self.state == State.error:
            self._widget.pb_error.setEnabled(True)
            self._widget.pb_error.setStyleSheet("background-color:rgb(245, 121, 0)")

    @Slot()
    def back(self):
        self._widget.tab.setCurrentIndex(0)

    @Slot()
    def start_pressed(self):
        try:
            init()
        except Exception:
            rospy.logwarn('Service not active.')

    @Slot()
    def home_pressed(self):
        if self.state == State.homing:
            self._widget.tab.setCurrentIndex(1)
        else:
            try:
                home()
            except Exception:
                rospy.logwarn('Service not active.')

    @Slot()
    def velocity_pressed(self):
        if self.state == State.cart_velocity:
            self._widget.tab.setCurrentIndex(2)
        else:
            try:
                start_velocity()
            except Exception as err:
                rospy.logwarn('Service not active.')

    @Slot()
    def position_pressed(self):
        if self.state == State.joint_position:
            self._widget.tab.setCurrentIndex(3)
        else:
            try:
                start_position()
            except Exception as err:
                rospy.logwarn('Service not active.')

    @Slot()
    def error_pressed(self):
        if self.state == State.error:
            self._widget.tab.setCurrentIndex(4)

    @Slot()
    def reset_pressed(self):
        try:
            reset_errors()
        except Exception:
            rospy.logwarn('Service not active.')

    @Slot()
    def stop_pressed(self):
        try:
            stop_trajectory()
        except:
            rospy.logwarn('Service not active.')

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # - save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # - restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
