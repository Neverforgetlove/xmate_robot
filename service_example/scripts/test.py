#!/usr/bin/env python
#encoding=utf-8

import rospy
import actionlib
from std_msgs.msg import String, Float32MultiArray, Int32
import time
import os
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from service_example.srv import Socket_srv

class rostopic_test():
    def __init__(self):

        rospy.init_node("rostopic_test",anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rate = rospy.get_param("~Rate", 10)
        self.rate = rospy.Rate(rate)

        self.bjj_move = False

        #启动Socket通信服务端
        self.ARV_Socket_server = rospy.Service('ARV_Socket_server', Socket_srv, self.ARV_Socket_handle)

        #客户端与Socket通信节点服务器连接
        rospy.wait_for_service("/Socket_Server")
        self.ARV_Socket_Client = rospy.ServiceProxy("Socket_Server", Socket_srv)
        rospy.loginfo('ARV_Socket_Client连接成功')

        self.pub = rospy.Publisher('agv_state', Int32, queue_size=1)
        self.Ass_pub = rospy.Publisher('AssStand_state', Int32, queue_size=1)

        rospy.Subscriber('PlaceDone', Int32, self.Place_Done)
        rospy.Subscriber('AssembleDone', Int32, self.Ass_Done)
  
    def ARV_Socket_handle(self,req):
        rospy.loginfo("收到PLC信息：%s" % req.request)
        if req.request == 'BJJ_SLSQ':
            self.bjj_move = True
            print("收到上料申请")
        if req.request == 'BJJ_XLSQ':
            print("收到上料申请")
        return 'OK'
    
    def Place_Done(self, data):
	    if data.data == 1:
            #rospy.sleep(5)
                response = self.ARV_Socket_Client('BJJ_SLWC')
                rospy.loginfo("收到PLC回复：%s" % response.response)
                if response.response == 'OK':
                    rospy.loginfo("上料完成")
    
    def Ass_Done(self, data):
        if data.data == 1:
            response = self.ARV_Socket_Client('BJJ_XLWC')
            rospy.loginfo("收到PLC回复：%s" % response.response)
            if response.response == 'OK':
                rospy.loginfo("下料完成")
    def shutdown(self):
        pass
if __name__ == "__main__":
    test = rostopic_test()
    rospy.spin()
