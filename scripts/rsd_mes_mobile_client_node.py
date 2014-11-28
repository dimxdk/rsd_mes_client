#!/usr/bin/env python

import rospy
import xmlrpclib
import datetime

from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_mobile_command, mes_mobile_status

class RSDMesMobileClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.ros_msg_command = mes_mobile_command()
        self.ros_msg_status = mes_mobile_status()
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_mobile_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_mobile_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_mobile_status, self.callbackStatus)

    def initVars(self):
        self.command_dict = {
            'COMMAND_WAIT': mes_mobile_command.COMMAND_WAIT, 
            'COMMAND_TIP': mes_mobile_command.COMMAND_TIP,
            'COMMAND_NAVIGATE' : mes_mobile_command.COMMAND_NAVIGATE,
            'COMMAND_ABORT' : mes_mobile_command.COMMAND_ABORT
        }
        self.command_error = mes_mobile_command.COMMAND_WAIT
        self.state_dict = {
            mes_mobile_status.STATE_WORKING : 'STATE_WORKING', 
            mes_mobile_status.STATE_FREE : 'STATE_FREE',
            mes_mobile_status.STATE_ERROR : 'STATE_ERROR'
        }
        self.state_error = 'STATE_ERROR'
        
    def getStatus(self):
        status = {
            'version_id': self.version_id,
            'robot_id': self.robot_id,
            'state': self.convertState(self.ros_msg_status.state),
            'time': str(datetime.datetime.fromtimestamp(self.ros_msg_status.header.stamp.to_time())),
            'battery': self.ros_msg_status.battery,
            'position': self.ros_msg_status.position,
            'status': self.ros_msg_status.status
        }
        return status
        
    def setCommand(self,command):
        self.ros_msg_command.command = self.convertCommand(command['command'])
        if (command.has_key("path")):
            self.ros_msg_command.path = command['path']
        else:
            self.ros_msg_command.path = ""
        return
    
    def fillDummyData(self):
        self.ros_msg_command.command = self.ros_msg_command.COMMAND_NAVIGATE
        self.ros_msg_command.path = ["path1","path2","destination_cell"]
    
    def fillDefaultData(self):
        self.ros_msg_command.command = self.ros_msg_command.COMMAND_WAIT
        self.ros_msg_command.path = []  
    
    def serverInfoExchange(self):
        try:
            status = self.getStatus()
            command = (self.server_connection.mobile_status(status))
            self.setCommand(command)            
        except:
            rospy.logerr("Communication with server failed")
            #self.online = False
        
if __name__ == '__main__':
    try:
        node_class = RSDMesMobileClientNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()