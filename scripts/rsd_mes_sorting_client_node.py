#!/usr/bin/env python

import rospy
from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_sorting_command, mes_sorting_status, mes_order, lego_brick

class RSDMesSortingClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.msg_command = mes_sorting_command()
        self.msg_status = mes_sorting_status()    
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_sorting_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_sorting_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_sorting_status, self.callbackStatus)
        
    def fillDummyData(self):
        self.msg_command.command = self.msg_command.COMMAND_SORTBRICKS
        self.msg_command.order.order_id = 1
        self.msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_RED, size=4, count=2))        
        self.msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_BLUE, size=6, count=5))

if __name__ == '__main__':
    try:
        node_class = RSDMesSortingClientNode()
        node_class.fillDummyData()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()