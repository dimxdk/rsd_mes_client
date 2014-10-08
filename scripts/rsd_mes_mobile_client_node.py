#!/usr/bin/env python

import rospy
from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_mobile_command, mes_mobile_status

class RSDMesMobileClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.msg_command = mes_mobile_command()
        self.msg_status = mes_mobile_status()    
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_mobile_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_mobile_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_mobile_status, self.callbackStatus)
        
    def fillDummyData(self):
        self.msg_command.command = self.msg_command.COMMAND_NAVIGATE
        self.msg_command.path = ["path1","path2","destination_cell"]
        
if __name__ == '__main__':
    try:
        node_class = RSDMesMobileClientNode()
        node_class.fillDummyData()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()