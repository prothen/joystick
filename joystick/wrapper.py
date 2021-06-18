#!/usr/bin/env python
""" ROS wrapper for Brunner CLS-E joystick. """

__copyright__ = "Copyright (C) 2021 Philipp Rothenhäusler"
__author__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__status__ = "Development"


import ffb
import attr
import enum
import yaml
import numpy
import rospy
import rospkg
import typing
import geometry_msgs.msg
import joystick_msgs.msg


class HapticMode(enum.IntEnum):
    Inactive = 0
    Autopilot = 1
    ExternalForce = 2
    Test = 3


@attr.s
class Wrapper:

    # Define configuration attributes
    _test = attr.ib(default=False, type=bool)
    _mode = attr.ib(default=HapticMode.Inactive, type=HapticMode)
    _config_name = attr.ib(default=None, type=typing.Optional[dict])
    _debug_is_enabled = attr.ib(default=None, type=typing.Optional[dict])

    _ffb_interface : typing.Optional[ffb.interface.Interface] = None

    # ROS interface configuration
    _rate : typing.Optional[rospy.Rate] = None

    _msg_status : typing.Optional[joystick_msgs.msg.Status] = None
    _msg_human_control : typing.Optional[geometry_msgs.msg.Vector3Stamped] = None

    _publisher_human_control : typing.Optional[rospy.Publisher] = None

    def __attrs_post_init__(self):
        rospy.init_node("placeholder")

        # Fetch parameters
        self._initialise_parameters()
        config = self._fetch_config_from_yaml()

        self._ffb_interface = ffb.interface.Interface(
            config=config,
            debug_is_enabled=self._debug_is_enabled
            )
        self._ffb_interface.connect()
        # Admit external force operation by default
        self._ffb_interface.deactivate_autopilot()

        # Rate for streaming human control input
        # TODO: Note that for external force mode this is upper bounded by the actuation frequency
        self._rate = rospy.Rate(config['frequency_desired_human_control'])

        msg = joystick_msgs.msg.Status()
        self._msg_status = msg

        msg = geometry_msgs.msg.Vector3Stamped()
        msg.header.frame_id = "base_link"
        self._msg_human_control = msg

        rospy.Subscriber("~haptic_feedback",
            joystick_msgs.msg.HapticControlStamped,
            self._cb_haptic_feedback, queue_size=1)

        self._publisher_status = rospy.Publisher(
            "~status",
            joystick_msgs.msg.Status,
            queue_size=1, tcp_nodelay=True)

        self._publisher_human_control = rospy.Publisher(
            "~control",
            geometry_msgs.msg.Vector3Stamped,
            queue_size=1, tcp_nodelay=True)

    def _initialise_parameters(self):
        self._test = rospy.get_param("~test")
        self._config_name = rospy.get_param("~config_name")
        self._debug_is_enabled = rospy.get_param("~debug_is_enabled")

    def _fetch_config_from_yaml(self):
        try:
            rospack = rospkg.RosPack()
            path = rospack.get_path('joystick')
            config_path = "/".join([path, "config", self._config_name])
            rospy.loginfo('Load configuration from: \n{}'.format(config_path))

            if config_path is not None:
                with open('{}.yaml'.format(config_path), 'r') as stream:
                    try:
                        config = yaml.safe_load(stream)
                    except yaml.YAMLError as e:
                        print('Configuration: Encountered\n', e)
                print('Loaded configuration file  successfully')
                return config
        except:
            raise RuntimeError()

    def _cb_haptic_feedback(self, msg):
        """ Forward desired haptic feedback command to ffb interface. """
        # (Possibly) Temporary test feature
        if self._test:
            return

        if msg.mode == msg.AUTOPILOT:
            if not self._mode is HapticMode.Autopilot:
                self._ffb_interface.enable_autopilot(True)
                self._mode = HapticMode.Autopilot
            self._ffb_interface.set_position(
                x=msg.control.x,
                y=msg.control.y)
        elif msg.mode == msg.EXTERNAL_FORCE:
            if not self._mode is HapticMode.ExternalForce:
                self._ffb_interface.enable_autopilot(False)
                self._mode = HapticMode.ExternalForce
            self._ffb_interface.actuate(
                x=msg.control.x,
                y=msg.control.y)

    def _publish_human_control(self):
        control = self._ffb_interface.get_position()
        if control is not None:

            msg = self._msg_human_control
            msg.header.stamp = rospy.Time.now()

            msg.vector.x = control[0]
            msg.vector.y = control[1]

            self._publisher_human_control.publish(msg)

    def _publish_status(self):
        m = self._msg_status
        s = self._ffb_interface.statistics
        sent = s['sent_commands']
        received = s['received_commands']
        external = s['external_actuation_messages']
        dropped = sent - received

        m.messages_sent = sent
        m.messages_received = received
        m.messages_dropped = dropped
        m.messages_external = external

        self._publisher_status.publish(m)

    def run(self):
        """ Execute main loop for actuation. """
        while not rospy.is_shutdown():
            if self._test:
                self._ffb_interface.actuate_test(rospy.Time.now().to_sec())

            self._ffb_interface.actuation_stream_is_active()

            self._publish_human_control()
            self._publish_status()
            self._rate.sleep()

    def exit(self):
        """ Execute exit routine. """
        self._ffb_interface.exit()

