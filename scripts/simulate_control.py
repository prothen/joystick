#!/usr/bin/env python
""" Simple test node to forward joystick to control topic. """


__author__ = "Philipp Rothenhäusler"
__version__ = "0.1"
__status__ = "Development"
__copyright__ = "Copyright 2021 Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"


import attr
import rospy
import typing
import sensor_msgs.msg
import geometry_msgs.msg


@attr.s
class ControlSimulator
    """ Conversion node from joystick to alternative control topic.

        Note:
            Simple conversion from sensor_msgs.msg.Joy to
            geometry_msg.msg.Vector3Stamped.

            The conversion between joystick and control is fixed
            to two dimensional XY joystick applications.

    """

    _publisher_control = attr.ib(default=None,
            type=typing.Optional[rospy.Publisher])

    _msg_control = attr.ib(default=None,
            type=typing.Optional[geometry_msgs.msg.Vector3Stamped])

    _last_control_stamp = attr.ib(default=None,
            type=typing.Optional[rospy.Time])

    # Upper bound for republishing frequency
    _frequency = attr.ib(default=100, type=float)
    _rate = attr.ib(default=None, type=typing.Optional[rospy.Rate])

    def __attrs_post_init__(self):
        rospy.init_node("placeholder")
        rospy.Subscriber("~joystick_input",
                sensor_msgs.msg.Joy)

        self._publisher_control = rospy.Publisher(
            "~control",
            geometry_msgs.msg.Vector3Stamped,
            queue_size=1,
            tcp_nodelay=True)

        c = geometry_msgs.msg.Vector3Stamped()
        self._msg_control = c

        self._rate = rospy.Rate(self._frequency)

    def _cb_joystick_input(self, msg: sensor_msgs.msg.Joy):
        """ Parse joystick input with fixed mapping. """
        self._msg_control.header.stamp = msg.header.stamp
        self._msg_control.vector.x = msg.axes[1]
        self._msg_control.vector.y = msg.axes[0]

    def _publish_control(self):
        """ Publish parsed control message. """
        self._publisher_control.publish(self._msg_control)

    def run(self):
        """ Execute main loop. """
        rospy.loginfo("Wait for reception of joystick command...")

        while self._msg_control.header.stamp is None:
            self._rate.sleep()

        rospy.loginfo("-> [x] Received first joystick command.")

        rospy.loginfo("Commence forwarding...")

        while not rospy.is_shutdown():
            self._publish_control()
            self_rate.sleep()


if __name__ == "__main__":
    control_simulator = ControlSimulator()
    control_simulator.run()
