#!/usr/bin/env python
# encoding=utf-8
# Version: 1.0
# Date: 2020-10-25
# Description: agv move code, python2

import rospy
import actionlib
from std_msgs.msg import String, Float32MultiArray, Int32, Int64
import time
import json
import threading
import requests
import os
import sys
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from service_example.srv import Socket_srv
from signal import signal, SIGPIPE, SIG_DFL

# 屏蔽SIGPIPE信号，不抛出异常
signal(SIGPIPE, SIG_DFL)
# 开启导航的地图和路径的文件路径
CURRENT_MAP_PATH = "/home/xiaoqiang/map_path.txt"
# 路径点的文件路径　 0-中转台　1-立体仓库　2-装配台
CURRENT_PATH_DATA = "/home/xiaoqiang/path_data.txt"
# 充电桩位置的文件路径
CHARGE_DATA_PATH = "/home/xiaoqiang/slamdb/dock_station.txt"    # 内容有三行 第一二行2个数代表x1 y1, x2 y2 第三行代表x y theta
                                                                # 充电位置：(x1+x2)/2, (y1+y2)/2,theta = theta

class test_agv_move():
    def __init__(self):

        rospy.init_node("AGV_Move", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rate = rospy.get_param("~Rate", 10)
        self.rate = rospy.Rate(rate)


        # １-中转台　2-中转台二号点 3-立体仓库　4-装配台
        self.way_point1 = []
        self.way_point2 = []
        self.way_point3 = []
        self.way_point4 = []


        # １-中转台　2-中转台二号点 3-立体仓库　4-装配台　中间点
        self.center_point1 = []
        self.center_point2 = []
        self.center_point3 = []
        self.center_point4 = []
        
        try:
            if os.path.exists(CURRENT_PATH_DATA):
                sz = os.path.getsize(CURRENT_PATH_DATA)
                if not sz:
                    rospy.logerr("CURRENT_PATH_DATA is empty!")
                    os._exit(0)
            else:
                os.mknod(CURRENT_PATH_DATA)
                rospy.logerr("CURRENT_PATH_DATA is not exists!")
                os._exit(0)
            
            file = open(CURRENT_PATH_DATA)
            dataMat = []
            target_num = 0
            for line in file.readlines():
                curLine = line.strip().split(" ")
                dataMat.append(list(map(int, curLine[0])))
		#print(list(map(int, curLine[0])[0]))
                if list(map(int, curLine[0]))[0] == 1:
                    target_num += 1
                floatLine = list(map(float, curLine[1:4]))
                dataMat.append(floatLine)
            file.close()
            count = 1
            for i in range(len(dataMat)):
                if len(dataMat[i]) == 1 and dataMat[i][0] == 0:
                    if count == 1:
                        self.center_point1.append(dataMat[i+1])
                    if count == 2:
                        self.center_point2.append(dataMat[i+1])
                    if count == 3:
                        self.center_point3.append(dataMat[i+1])
                    if count == 4:
                        self.center_point4.append(dataMat[i+1])
                    i += 2
                    continue

                if len(dataMat[i]) == 1 and dataMat[i][0] == 1:
                    if count == 1:
                        self.way_point1.append(dataMat[i+1])
                    if count == 2:
                        self.way_point2.append(dataMat[i+1])
                    if count == 3:
                        self.way_point3.append(dataMat[i+1])
                    if count == 4:
                        self.way_point4.append(dataMat[i+1])
                    i += 2
                    count += 1
                    continue
            rospy.loginfo("get point success")
            print(self.center_point1, self.center_point2, self.center_point3)
        except IOError:
            rospy.logerr("File is not accessible")
            os._exit(0)
        

        self.map_path_data = []
        try:
            if os.path.exists(CURRENT_MAP_PATH):
                sz = os.path.getsize(CURRENT_MAP_PATH)
                if not sz:
                    rospy.logerr("CURRENT_MAP_PATH is empty!")
                    os._exit(0)
            else:
                os.mknod(CURRENT_MAP_PATH)
                rospy.logerr("CURRENT_MAP_PATH is not exists!")
                os._exit(0)

            file = open(CURRENT_MAP_PATH)
            for line in file.readlines():
                curLine = line.strip().split(" ")
                floatLine = map(str, curLine)
                self.map_path_data.append(floatLine)
            if len(self.map_path_data) == 1:
                rospy.logerr("Data error!!")
                os._exit(0)
            file.close()
        except IOError:
            rospy.logerr("File is not accessible")
            os._exit(0)

        # 控制状态 0：语音导航 1：全自动
        self.control_type = rospy.get_param('~control_type')

        #启动Socket通信服务端
        self.ARV_Socket_server = rospy.Service('ARV_Socket_server', Socket_srv, self.ARV_Socket_handle)

        #客户端与Socket通信节点服务器连接
        if self.control_type == 1:
            rospy.wait_for_service("/Socket_Server")
            self.ARV_Socket_Client = rospy.ServiceProxy("Socket_Server", Socket_srv)
            rospy.loginfo('ARV_Socket_Client连接成功')

        self.pub = rospy.Publisher('agv_state', Int32, queue_size=1)
        self.Ass_pub = rospy.Publisher('AssStand_state', Int32, queue_size=1)

        rospy.Subscriber('GraspDone', Int32, self.Grasp_Done)
        rospy.Subscriber('PlaceDone', Int32, self.Place_Done)
        rospy.Subscriber('AssembleDone', Int32, self.Ass_Done)
        rospy.Subscriber('nav_position', Int64, self.nav_move_controller)
        rospy.Subscriber('Start_State', Int32, self.Get_Start_State)

        self.mission_start = False

        # 导航地图和路径
        self.nav_map = self.map_path_data[0]
        self.nav_path = self.map_path_data[1]
        print(self.nav_map)
        rospy.loginfo("start map path: [%s] [%s]", str(self.nav_map), str(self.nav_path))
    #rospy.loginfo(str(self.nav_path))

        self.move_nub = 0
        self.system_status = None      # 系统状态
        self.start_nav_status = False   # 是否开启导航
        self.agv_charging = False       # 是否在充电
        self.move_timeout = 90         # 任务等待时间

        self.bjj_move = False

        # 充电桩位置
        self.charge_x = None
        self.charge_y = None
        self.charge_theta = None

        # 自动充电动作的id
        self.charge_id = None

        # agv当前在哪个点
        self.current_point = -1

        self.current_map = None   # 获取当前导航的地图和路径
        self.current_path = None

        # 获取当前位置
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_pose_angle = None

        # 系统基本信息：　电量　        rgb摄像头　深度摄像头　编码器　IMU
        # 数据意义：     电量百分比　　　0 代表异常　非０代表正常
        # 每次发布导航点先需要先更新数据
        self.system_state_data = [0, 0, 0, 0, 0]

        # 每次开启首先更新系统当前状态和基本信息
        self.get_sys_status()
        self.get_sys_state_data()
        
        if self.start_nav_status == False:
            self.start_agv_nav()
            self.sys_status = "Navigating"
            self.start_nav_status = True
            time.sleep(10)

    def Get_Start_State(self, data):
        if data.data == 1:
            print("Start mission")
            self.mission_start = True 
            
    def nav_move_controller(self, data):
        # 去物料台
        if data.data == 0:
            self.agv_move(0)
        # 去立体仓库
        if data.data == 1:
            self.agv_move(2)
        # 去装配台
        if data.data == 2:
            self.agv_move(3)
        # 开始巡逻
        if data.data == 3:
            if self.agv_move(0):
                if self.agv_move(2):
                    if self.agv_move(3):
                        rospy.loginfo("巡逻结束")
        # 语音控制抓取 需要开启place_grasp_demo_new.launch,需要修改该launch文件中的节点
        if data.data == 4:
            if self.agv_move(0):
                self.pub.publish(1)
                rospy.loginfo("语音控制抓取物料")
        # 语音控制放置 需要开启place_grasp_demo_new.launch,需要修改该launch文件中的节点
        if data.data == 5:
            if self.agv_move(3):
                self.pub.publish(3)
                rospy.loginfo("语音控制放置物料")
        # 语音控制放成品 需要开启product_place_test_demo.launch
        if data.data == 6:
            if self.agv_move(2):
                self.pub.publish(3)
                rospy.loginfo("语音控制放置成品")

    def ARV_Socket_handle(self,req):
        rospy.loginfo("收到PLC信息：%s" % req.request)
        if req.request == 'SL_SY' or req.request == 'XL_SY':
            rospy.loginfo("get true msg")
        else:
            print("get PLC msg，but is warning msg：", req.request)
            print(len(req.request) - len('SL_SY'))
        if req.request == 'SL_SY':
            self.get_bjj_type(1)
            print("current bjj_move type is: ", self.bjj_move)
            return 'OK'
        if req.request == 'XL_SY':
            self.Ass_pub.publish(1)
            return 'OK'

    def get_bjj_type(self, type):
        if type == 1:
            self.bjj_move = True
            return True
        else:
            return False

    def Ass_Done(self, data):
        if data.data == 1:
            # response = self.ARV_Socket_Client('BJJ_XLWC')
            # rospy.loginfo("收到PLC回复：%s" % response.response)
            # if response.response == 'OK':
            #     rospy.loginfo("下料完成")
            print("get finished product  success! , move to LTCK")
            self.agv_move(0)
            if self.agv_move(2):
                self.pub.publish(4)
                rospy.loginfo("done")
        else:
            print("get Ass_done state fail, Ass_Done data is: ", data.data)

    def Grasp_Done(self, data):
        if data.data == 2 and self.control_type == 1:
            if self.agv_move(1):
                self.pub.publish(2)

        if data.data == 1 and self.control_type == 1:
            self.agv_move(2)
            if self.agv_move(3):
                print("run here!!, data is :", self.bjj_move)
                if self.bjj_move or self.get_bjj_type(1):
                    self.pub.publish(3)
                    self.bjj_move = False
        
    def Place_Done(self, data):
        if data.data == 1 and self.control_type == 1:
            rospy.sleep(5)
            response = self.ARV_Socket_Client('SL_SYWC')
            rospy.loginfo("收到PLC回复：%s" % response.response)
            if response.response == 'OK':
                rospy.loginfo("上料完成")
            # self.agv_move(0)
            # self.agv_move(1)
            # self.pub.publish(3)

    # 获取当前系统状态
    def get_sys_status(self):
        r = requests.get('http://127.0.0.1:3546/api/v1/system/status', timeout=5)
        if r.status_code == requests.codes.ok:
            sys_status = r.json()
            if sys_status["status"]:
                rospy.loginfo("current system status： %s" % str(sys_status["status"]))
                self.system_status = sys_status["status"]
            if self.system_status == "Navigating":
                self.start_nav_status = True
            return sys_status["status"]
        else:
            rospy.logerr(r)
            return "get system status error"

    # 获取当前系统的基本信息
    def get_sys_state_data(self):
        r = requests.get('http://127.0.0.1:3546/api/v1/system/info', timeout=5)
        if r.status_code == requests.codes.ok:
            sys_state_data = r.json()
            self.system_state_data[0] = sys_state_data["battery"]
            self.system_state_data[1] = sys_state_data["camera_rgb"]
            self.system_state_data[2] = sys_state_data["camera_depth"]
            self.system_state_data[3] = sys_state_data["odom"]
            self.system_state_data[4] = sys_state_data["imu"]

        # 电量少于45% 首先执行充电 当电量>90%后再恢复导航状态
        '''
        if self.system_state_data[0] < 45:
            if self.go_to_charge() and self.agv_charging == False:
                self.go_to_charge()
                if self.system_status == "Charging":
                    self.agv_charging = True
            while self.system_state_data[0] < 90:
                self.get_sys_state_data()
                time.sleep(2.0)
            self.start_agv_nav()
            self.agv_charging = False
        '''

        if any(self.system_state_data[1:4]) == 0:
            rospy.logerr("system state error!!!")
            return False

        return True
    
    # 获取导航的地图和路径
    def get_nav_data(self):
        if self.start_nav_status:
            get_nav_map_path = requests.get('http://127.0.0.1:3546/api/v1/navigation/current_path', timeout=5)
            if get_nav_map_path.status_code == requests.codes.ok:
                data = get_nav_map_path.json()
                self.current_map = data["map"]
                self.current_path = data["path"]
                rospy.loginfo("current_map_name: %s" % self.current_map)
                rospy.loginfo("current_path_name: %s" % self.current_path)
                return True
            else:
                rospy.logwarn(get_nav_map_path.text)
                return False
        else:
            rospy.logwarn("system status must be in Navigating!!")
            return False
    
    # 停止导航
    def stop_agv_nav(self):
        if self.start_nav_status:
            stop_nav = requests.get('http://127.0.0.1:3546/api/v1/navigation/stop', timeout=10)
            if stop_nav.status_code == requests.codes.ok:
                data = stop_nav.json()
                rospy.loginfo(data)
                stop_nav_status = data["result"]
                if stop_nav_status:
                    self.start_nav_status = False
                    rospy.loginfo("stop nav success")
                    return True
                else:
                    rospy.logwarn("stop nav fail")
                    return False
            else:
                rospy.logerr(stop_nav)
                return False
        else:
            rospy.loginfo("当前系统未开启导航")
            return False

    # 切换系统状态至导航状态，开启导航
    def start_agv_nav(self):
        if self.start_nav_status == False:
            agv_nav_data = {"map": self.nav_map, "path": self.nav_path}
            r = requests.get('http://127.0.0.1:3546/api/v1/navigation/start', params=agv_nav_data, timeout=10)
            if r.status_code == requests.codes.ok:
                start_nav_res = r.json()
                if start_nav_res["result"]:
                    self.start_nav_status = True
                    if self.get_nav_data():
                        rospy.loginfo("当前导航地图和路径为： [%s] [%s]", str(self.current_map), str(self.current_path))
                        if self.current_map == self.nav_map and self.current_path == self.nav_path:
                            rospy.loginfo("start nav success")
                        else:
                            self.start_nav_status = False
                            rospy.logwarn("start nav fail")
                else:
                    rospy.logerr("开启导航状态失败")
            else:
                rospy.logerr("请求开启导航失败，返回： ", r.text)
        else:
            rospy.logwarn("当前系统状态已经处于导航状态")
            return False
    
    # 获取agv当前位置
    def get_agv_pose(self):
        pose = requests.get('http://127.0.0.1:3546/api/v1/navigation/pose', timeout=5)
        if pose.status_code == requests.codes.ok:
            pose_data = pose.json()
            self.current_pose_x = pose_data["x"]
            self.current_pose_y = pose_data["y"]
            self.current_pose_angle = pose_data["angle"]
            rospy.loginfo("current pose： [%lf] [%lf] [%lf]", self.current_pose_x , self.current_pose_y, self.current_pose_angle)
            return True
        else:
            rospy.logerr("get pose fail", pose)
            return False

    # 查询任务id的信息，返回任务id的执行状态
    def get_action_state(self, action_id):
        action_data = {"id": action_id}
        get_action_state = requests.get("http://127.0.0.1:3546/api/v1/task", params=action_data)
        if get_action_state.status_code == requests.codes.ok:
            action_state_data = get_action_state.json()
            return action_state_data["state"]
        else:
            rospy.logerr(get_action_state.text)
            return "get action state error"

    # 开始任务
    def start_action(self, action_id):
        action_data = {"id": action_id}
        start_action_state = requests.get("http://127.0.0.1:3546/api/v1/task/start", params=action_data)
        if start_action_state.status_code == requests.codes.ok:
            start_action_state_data = start_action_state.json()
            if start_action_state_data["state"] == "WORKING":
                return True
        return False

    # 暂停任务
    def pause_action(self, action_id):
        action_data = {"id": action_id}
        pause_action_state = requests.get("http://127.0.0.1:3546/api/v1/task/pause", params=action_data)
        if pause_action_state.status_code == requests.codes.ok:
            pause_action_state_data = pause_action_state.json()
            if pause_action_state_data["state"] == "PAUSED":
                return True
        return False
    
    # 取消任务
    def stop_action(self, action_id):
        action_data = {"id": action_id}
        stop_action_state = requests.get("http://127.0.0.1:3546/api/v1/task/stop", params=action_data)
        if stop_action_state.status_code == requests.codes.ok:
            stop_action_state_data = stop_action_state.json()
            if stop_action_state_data["state"] == "CANCELLED":
                return True
        return False

    # 继续任务
    def resume_action(self, action_id):
        action_data = {"id": action_id}
        resume_action_state = requests.get("http://127.0.0.1:3546/api/v1/task/resume", params=action_data)
        if resume_action_state.status_code == requests.codes.ok:
            resume_action_state_data = resume_action_state.json()
            if resume_action_state_data["state"] == "WORKING":
                return True
        return False
    
    # 计算当前位置和目标位置的偏差
    # xy不得大于30mm,角度不得大于10°
    def compute_deviation(self):
        
        # 更新当前位置
        self.get_agv_pose()
        
        x_deviation = self.current_pose_x - self.way_point[self.move_nub][0]
        y_deviation = self.current_pose_y - self.way_point[self.move_nub][1]
        angle_deviation = self.current_pose_angle - self.way_point[self.move_nub][2]

        if abs(x_deviation) > 0.03 or abs(y_deviation)>0.03 or abs(math.degrees(angle_deviation)) > 10:
            return False
        return True
    
    # 中间点
    def agv_move_center_point(self,move_num):
        
        if move_num == 1:
            for i in range(len(self.center_point1)):

                agv_move_id = None
                try_time = self.move_timeout

                agv_point = {"x": self.center_point1[i][0],
                         "y": self.center_point1[i][1],
                         "theta": math.radians(self.center_point1[i][2])}
                r = requests.post('http://127.0.0.1:3546/api/v1/navigation/start_nav_task', json=agv_point, timeout=5)
                if r.status_code == requests.codes.ok:
                    agv_move_res = r.json()
                    agv_move_id = agv_move_res["id"]
                else:
                    rospy.logerr("post nav task fail!")
                    return False
                nav_move_state = self.get_action_state(agv_move_id)

                while nav_move_state != "COMPLETE" and try_time > 0:
                    nav_move_state = self.get_action_state(agv_move_id)
                    if nav_move_state == "COMPLETE":
                        rospy.loginfo("nav move success")
                        time.sleep(1)
                        break
                    else:
                        rospy.loginfo("nav move state: %s" % nav_move_state)
                    time.sleep(1.0)
                    try_time -= 1
                if try_time == 0:
                    self.stop_action(agv_move_id)
                
        if move_num == 2:
            for i in range(len(self.center_point2)):

                agv_move_id = None
                try_time = self.move_timeout

                agv_point = {"x": self.center_point2[i][0],
                         "y": self.center_point2[i][1],
                         "theta": math.radians(self.center_point2[i][2])}
                r = requests.post('http://127.0.0.1:3546/api/v1/navigation/start_nav_task', json=agv_point, timeout=5)
                if r.status_code == requests.codes.ok:
                    agv_move_res = r.json()
                    agv_move_id = agv_move_res["id"]
                else:
                    rospy.logerr("post nav task fail!")
                    return False
                nav_move_state = self.get_action_state(agv_move_id)

                while nav_move_state != "COMPLETE" and try_time > 0:
                    nav_move_state = self.get_action_state(agv_move_id)
                    if nav_move_state == "COMPLETE":
                        rospy.loginfo("nav move success")
                        time.sleep(1)
                        break
                    else:
                        rospy.loginfo("nav move state: %s" % nav_move_state)
                    time.sleep(1.0)
                    try_time -= 1
                if try_time == 0:
                    self.stop_action(agv_move_id)

        if move_num == 3:
            for i in range(len(self.center_point3)):

                agv_move_id = None
                try_time = self.move_timeout

                agv_point = {"x": self.center_point3[i][0],
                         "y": self.center_point3[i][1],
                         "theta": math.radians(self.center_point3[i][2])}
                r = requests.post('http://127.0.0.1:3546/api/v1/navigation/start_nav_task', json=agv_point, timeout=5)
                if r.status_code == requests.codes.ok:
                    agv_move_res = r.json()
                    agv_move_id = agv_move_res["id"]
                else:
                    rospy.logerr("post nav task fail!")
                    return False
                nav_move_state = self.get_action_state(agv_move_id)

                while nav_move_state != "COMPLETE" and try_time > 0:
                    nav_move_state = self.get_action_state(agv_move_id)
                    if nav_move_state == "COMPLETE":
                        rospy.loginfo("nav move success")
                        time.sleep(1)
                        break
                    else:
                        rospy.loginfo("nav move state: %s" % nav_move_state)
                    time.sleep(1.0)
                    try_time -= 1
                if try_time == 0:
                    self.stop_action(agv_move_id)

        if move_num == 4:
            for i in range(len(self.center_point4)):

                agv_move_id = None
                try_time = self.move_timeout

                agv_point = {"x": self.center_point4[i][0],
                         "y": self.center_point4[i][1],
                         "theta": math.radians(self.center_point4[i][2])}
                r = requests.post('http://127.0.0.1:3546/api/v1/navigation/start_nav_task', json=agv_point, timeout=5)
                if r.status_code == requests.codes.ok:
                    agv_move_res = r.json()
                    agv_move_id = agv_move_res["id"]
                else:
                    rospy.logerr("post nav task fail!")
                    return False
                nav_move_state = self.get_action_state(agv_move_id)

                while nav_move_state != "COMPLETE" and try_time > 0:
                    nav_move_state = self.get_action_state(agv_move_id)
                    if nav_move_state == "COMPLETE":
                        rospy.loginfo("nav move success")
                        time.sleep(1)
                        break
                    else:
                        rospy.loginfo("nav move state: %s" % nav_move_state)
                    time.sleep(1.0)
                    try_time -= 1
                if try_time == 0:
                    self.stop_action(agv_move_id)

        return True

    # 发布移动点
    def agv_move(self,waypoint=-1):

        agv_move_id = None
        try_time = self.move_timeout
        self.move_nub = waypoint
        # 需要添加电量判断，每次移动前获取系统基本信息，确保电量足够
        # 电量百分比低于45% 执行自动充电
        if self.move_nub < 4 and self.get_sys_state_data():
            if self.move_nub == 0:
                if len(self.center_point1):
                    self.agv_move_center_point(1)
                agv_point = {"x": self.way_point1[0][0],
                         "y": self.way_point1[0][1],
                         "theta": math.radians(self.way_point1[0][2])}

            if self.move_nub == 1:
                if len(self.center_point2):
                    self.agv_move_center_point(2)
                agv_point = {"x": self.way_point2[0][0],
                         "y": self.way_point2[0][1],
                         "theta": math.radians(self.way_point2[0][2])}

            if self.move_nub == 2:
                if len(self.center_point3):
                    self.agv_move_center_point(3)
                agv_point = {"x": self.way_point3[0][0],
                         "y": self.way_point3[0][1],
                         "theta": math.radians(self.way_point3[0][2])}

            if self.move_nub == 3:
                if len(self.center_point4):
                    self.agv_move_center_point(4)
                agv_point = {"x": self.way_point4[0][0],
                         "y": self.way_point4[0][1],
                         "theta": math.radians(self.way_point4[0][2])}

            rospy.loginfo(agv_point)
            r = requests.post('http://127.0.0.1:3546/api/v1/navigation/start_nav_task', json=agv_point, timeout=5)
            if r.status_code == requests.codes.ok:
                agv_move_res = r.json()
                agv_move_id = agv_move_res["id"]
            else:
                rospy.logerr("post nav task fail!")
                return False
        
        nav_move_state = self.get_action_state(agv_move_id)

        # 监听60秒，是否成功
        while nav_move_state != "COMPLETE" and try_time > 0:
            nav_move_state = self.get_action_state(agv_move_id)
            if nav_move_state == "COMPLETE":
                rospy.loginfo("nav move success")
                time.sleep(1)
                # 收到返回成功后，判断和目标点的误差是否超过一定范围
                # 超过一定范围则再次发送该目标点
                #if self.compute_deviation():
                    #pub.publish("complete")
                return True
                break
                #else:
                #    self.agv_move(waypoint)
            else:
                rospy.loginfo("nav move state: %s" % nav_move_state)
            time.sleep(1.0)
            try_time -= 1
        
        # 超时未完成则取消任务
        self.stop_action(agv_move_id)
        return False
    
    # 自动充电,位置由客户端定义的充电桩位置，从文件中读取
    # 执行自动充电成功后，需要监听系统电量判断是否充电完成
    def go_to_charge(self):
        try_time = self.move_timeout
        #if self.system_status != "Navigating":
        #    self.start_agv_nav()
         #   time.sleep(2.0)

        # 从本地地图中读取充电桩位置
        data = []
        try:
            file = open(CHARGE_DATA_PATH)
            for line in file.readlines():
                curLine = line.strip().split(" ")
                floatLine = map(float, curLine)  # python3.x -> list(map(float, curLine))
                data.append(floatLine)
            file.close()
        except IOError:
            rospy.logerr("File is not accessible")
            return False

        self.charge_x = (data[0][0] + data[1][0]) / 2
        self.charge_y = (data[0][1] + data[1][1]) / 2
        self.charge_theta = data[2][2]

        action_data = {"type": "charge_action", "x": self.charge_x, "y": self.charge_y, "theta": self.charge_theta}
        r = requests.post('http://127.0.0.1:3546/api/v1/action', json=action_data, timeout=5)
        if r.status_code == requests.codes.ok:
            charge_data = r.json()
            charge_action_id = charge_data["id"]
            self.charge_id = charge_action_id
            charge_action_state = charge_data["state"]
            rospy.loginfo("charge state: %s" % charge_action_state)
            self.start_action(self.charge_id)
        while charge_action_state != "COMPLETE" and try_time > 0:
            charge_action_state = self.get_action_state(charge_action_id)
            rospy.loginfo("charge state: %s" % charge_action_state)
            if charge_action_state == "COMPLETE":
                rospy.loginfo("charge move success")
                time.sleep(1)
                self.agv_charging = True
                return True
            else:
                rospy.loginfo("charge move state: %s" % charge_action_state)
            time.sleep(1.0)
            try_time -= 1
        return False

    # 监听充电时的电量，达到100%后停止充电，系统恢复到free状态
    # 监听完成后停止充电，小车会自动往前离开充电桩
    # 离开充电桩后开启导航，但是必须要确保开启导航后能够马上定位当前位置
    # 防止左右摆动定位自身对充电桩产生碰撞
    def monitor_charge(self, id):
        self.get_sys_state_data()
        while self.system_state_data[0] != 100:
            self.get_sys_state_data()
        return self.stop_action(id)
    
    def start(self):
        if self.control_type == 0:
            pass
        if self.control_type == 1:
            rospy.wait_for_message("Start_State", Int32)
            if self.mission_start == True:
                if self.agv_move(0):
                    self.pub.publish(1)

    def shutdown(self):
        pass


if __name__ == "__main__":
    test = test_agv_move()
    test.start_agv_nav()
    test.get_nav_data()
    test.start()
    rospy.spin()
