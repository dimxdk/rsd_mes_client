#!/usr/bin/env python

import rospy
import xmlrpclib
import datetime

from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_sorting_command, mes_sorting_status, mes_order, lego_brick

class RSDMesSortingClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.ros_msg_command = mes_sorting_command()
        self.ros_msg_status = mes_sorting_status()    
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_sorting_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_sorting_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_sorting_status, self.callbackStatus)
          
    def initVars(self):
        self.command_dict = {
            'COMMAND_WAIT': mes_sorting_command.COMMAND_WAIT,
            'COMMAND_LOADBRICKS': mes_sorting_command.COMMAND_LOADBRICKS,
            'COMMAND_SORTBRICKS' : mes_sorting_command.COMMAND_SORTBRICKS,
            'COMMAND_ABORT' : mes_sorting_command.COMMAND_ABORT
        }
        self.command_error = mes_sorting_command.COMMAND_WAIT
        self.state_dict = {
            mes_sorting_status.STATE_FREE : 'STATE_FREE',
            mes_sorting_status.STATE_LOADING : 'STATE_LOADING',
            mes_sorting_status.STATE_ORDERSORTED : 'STATE_ORDERSORTED',
            mes_sorting_status.STATE_OUTOFBRICKS : 'STATE_OUTOFBRICKS',
            mes_sorting_status.STATE_SORTING : 'STATE_SORTING',
            mes_sorting_status.STATE_ERROR : 'STATE_ERROR'
        }
        self.state_error = 'STATE_ERROR'     
     
    def getStatus(self):
        status = {
            'version_id': self.version_id,
            'robot_id': self.robot_id,
            'state': self.convertState(self.ros_msg_status.state),
            'time': str(datetime.datetime.fromtimestamp(self.ros_msg_status.header.stamp.to_time())),
            'done_pct': self.ros_msg_status.done_pct,
            'status': self.ros_msg_status.status
        }
        return status 
                
    def setCommand(self,command):
        self.ros_msg_command.command = self.convertCommand(command['command'])
        if (command.has_key("order")):
            self.msg_command.order.order_id = command['order']['order_id']
            legos = command['order']['bricks']
            self.msg_command.order.bricks = []
            for i in range(len(legos)):
                self.msg_command.order.bricks.append(lego_brick())
                self.msg_command.order.bricks[i].color = legos[i]['color']
                self.msg_command.order.bricks[i].size = legos[i]['size']
                self.msg_command.order.bricks[i].count = legos[i]['count']
        else:
            self.ros_msg_command.order.order_id = 0
            self.ros_msg_command.order.bricks = []
        return
    
    def fillDefaultData(self):
        self.ros_msg_command.command = self.ros_msg_command.COMMAND_ABORT
        self.ros_msg_command.order.order_id = 0
        self.ros_msg_command.order.bricks = []
    
    def fillDummyData(self):
        self.ros_msg_command.command = self.ros_msg_command.COMMAND_SORTBRICKS
        self.ros_msg_command.order.order_id = 1
        self.ros_msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_RED, size=4, count=2))        
        self.ros_msg_command.order.bricks.append(lego_brick(color=lego_brick.COLOR_BLUE, size=6, count=5))

    def serverInfoExchange(self):
        #try:
            status = self.getStatus()
            command = (self.server_connection.cell_status(status))
            self.setCommand(command)            
        #except:
            #rospy.logerr("Communication with server failed")
            #self.online = False

if __name__ == '__main__':
    try:
        node_class = RSDMesSortingClientNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()