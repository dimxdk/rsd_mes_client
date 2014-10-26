#!/usr/bin/env python
__author__ = 'armienn'

import rospy
from rsd_mes_client.msg import mes_sorting_command, mes_sorting_status, lego_brick, mes_order

def publish_loop():
    rospy.init_node('sortingdummy', anonymous=True)
    msg_status = mes_sorting_status()
    msg_status.header.stamp = rospy.Time.now()
    msg_status.version_id = 1
    msg_status.state = msg_status.STATE_FREE
    msg_status.done_pct = 0
    msg_status.status = 'Hi'

    pub = rospy.Publisher('/mes/status', mes_sorting_status)
    while not rospy.is_shutdown():
        pub.publish(msg_status)
        response = raw_input('Input something')
        print response


if __name__ == '__main__':
    try:
        publish_loop()
    except rospy.ROSInterruptException:
        print 'damn'
        pass
