#!/usr/bin/env python

import rospy


import joystick


if __name__ == "__main__":
    wrapper = joystick.wrapper.Wrapper()
    try:
        wrapper.run()
    except rospy.ROSInterruptException as e:
        print('Encountered exception!:\n', e)
        wrapper.exit()
