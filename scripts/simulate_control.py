#!/usr/bin/env python
""" Simple test node to forward joystick to control topic.

    Note:
        The control topic corresponds to the joystick position as

            - /joystick/control
            - geometry_msgs.msg.Vector3Stamped

        while the haptic feedback signal is under

            - /joystick/haptic_feedback
            - joystick_msgs.msg.HapticControlStamped

"""


__author__ = "Philipp Rothenhäusler"
__version__ = "0.1"
__status__ = "Development"
__copyright__ = "Copyright 2021 Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"


import attr
import rospy
import numpy
import typing
import sensor_msgs.msg
import geometry_msgs.msg
import joystick_msgs.msg


@attr.s
class ControlSimulator:
    """ Conversion node from joystick to alternative control topic.

        Note:
            Simple conversion from sensor_msgs.msg.Joy to
            geometry_msg.msg.Vector3Stamped.

            The conversion between joystick and control is fixed
            to two dimensional XY joystick applications.

    """
    # ROS publishers
    _publisher_control = attr.ib(default=None,
            type=typing.Optional[rospy.Publisher])

    _publisher_haptic_feedback = attr.ib(default=None,
            type=typing.Optional[rospy.Publisher])

    # ROS messages
    _msg_control = attr.ib(default=None,
            type=typing.Optional[geometry_msgs.msg.Vector3Stamped])

    _msg_haptic_feedback = attr.ib(default=None,
            type=typing.Optional[joystick_msgs.msg.HapticControlStamped])

    # Upper bound for republishing frequency
    _frequency = attr.ib(default=100, type=float)
    _rate = attr.ib(default=None, type=typing.Optional[rospy.Rate])

    # Deadzone to consider zero input _d : [-th, th] -> 0
    # TODO: Rescale output
    _deadzone_threshold = attr.ib(default=.2, type=float)

    def __attrs_post_init__(self):
        rospy.init_node("placeholder")
        rospy.Subscriber("~joystick_input",
                sensor_msgs.msg.Joy,
                self._cb_joystick_input)

        self._publisher_control = rospy.Publisher(
            "~control",
            geometry_msgs.msg.Vector3Stamped,
            queue_size=1,
            tcp_nodelay=True)

        self._publisher_haptic_feedback = rospy.Publisher(
            "~haptic_feedback",
            joystick_msgs.msg.HapticControlStamped,
            queue_size=1,
            tcp_nodelay=True)

        c = geometry_msgs.msg.Vector3Stamped()
        self._msg_control = c

        c = joystick_msgs.msg.HapticControlStamped()
        c.mode = joystick_msgs.msg.HapticControlStamped.EXTERNAL_FORCE
        self._msg_haptic_feedback = c

        self._rate = rospy.Rate(self._frequency)

    def _deadzone(self, value):
        """ Apply joystick deadzone. """
        return 0 if numpy.abs(value) < self._deadzone_threshold else value

    def _cb_joystick_input(self, msg: sensor_msgs.msg.Joy):
        """ Parse joystick input with fixed mapping. """
        self._msg_control.header.stamp = msg.header.stamp
        self._msg_control.vector.x = self._deadzone(msg.axes[1])
        self._msg_control.vector.y = self._deadzone(msg.axes[0])

        # Apply joystick input as haptic feedback output
        self._msg_haptic_feedback.stamp = msg.header.stamp
        self._msg_haptic_feedback.control = self._msg_control.vector

    def _publish_control(self):
        """ Publish parsed control message. """
        self._publisher_control.publish(self._msg_control)

    def _publish_haptic_feedback(self):
        """ Publish parsed control message. """
        self._publisher_haptic_feedback.publish(self._msg_haptic_feedback)

    def run(self):
        """ Execute main loop. """
        rospy.loginfo("Wait for reception of joystick command...")

        while self._msg_control.header.stamp is None:
            self._rate.sleep()

        rospy.loginfo("-> [x] Received first joystick command.")

        rospy.loginfo("Commence forwarding...")

        while not rospy.is_shutdown():
            self._publish_control()
            self._publish_haptic_feedback()
            self._rate.sleep()


if __name__ == "__main__":
    control_simulator = ControlSimulator()
    control_simulator.run()
