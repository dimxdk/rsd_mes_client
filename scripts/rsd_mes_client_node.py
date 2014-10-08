#!/usr/bin/env python

import rospy

class RSDMesClientNode():
    def __init__(self):
        self.version_id = 1
        self.getParams()
        self.initMsg()
        self.initNode()
        self.initTopic()
        self.initTimer()
        
    def getParams(self):
        self.mes_command_topic = rospy.get_param("mes_command_topic", 'mes_command_topic') 
        self.mes_status_topic = rospy.get_param("mes_status_topic", 'mes_status_topic')
        self.update_duration = rospy.get_param("update_duration", 1.0)

    def initMsg(self):
        return
        
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_generic_client')
        
    def initTopic(self):
        return

    def initTimer(self):
        rospy.Timer(rospy.Duration(self.update_duration), self.update)  
        
    def callbackStatus(self,status):
        return

    def publishCommand(self):        
        try:
            self.msg_command.header.stamp = rospy.Time.now()
            self.mes_command_publisher.publish(self.msg_command)
        except:
            rospy.logerr("Publisher/messages not initialized correct!")
    
    def update(self, event):
        self.publishCommand()
        

if __name__ == '__main__':
    print("This file is not intended to run as standalone")