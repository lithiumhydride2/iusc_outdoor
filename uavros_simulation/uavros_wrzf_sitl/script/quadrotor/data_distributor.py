#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: data_distributor.py
Date: 2021/7/9 17:48
LastEditors: LI Jinjie
LastEditTime: 2021/7/9 17:48
Description: 专门进行监听到数据后的处理。
'''
import json
import time
import sys

import quadrotor.parameters as para
import quadrotor.cmd as cmd
from send_to_fk import *
from cmd_with_outer_hardware import *

class DataDistributor:
    def __init__(self, uav, data_2_fk, decision_maker=None):
        # 中心节点传入两个类，其他节点只传入一个类
        self.uav = uav
        self.decision_maker = decision_maker
        self.data_2_fk = data_2_fk

        # 用于信息接收
        self.rx_node = None
        self.rx_cmd = None
        self.rx_position = [0, 0, 0]

        # 用于切出循环
        self.break_flag = False

    def callback(self, xbee_message):
        print("From %s >> %s" % (xbee_message.remote_device.get_node_id(),
                                 xbee_message.data.decode()))
        data_rx = json.loads(xbee_message.data.decode())

        self.rx_node = data_rx[cmd.KEY_NODE]
        self.rx_cmd = data_rx[cmd.KEY_CMD]
        if cmd.KEY_POSITION in data_rx:
            self.rx_position = data_rx[cmd.KEY_POSITION]

        # 命令处理
        if self.uav.name == para.UAV_AC_NAME:  # 中心节点

            if self.rx_node == self.uav.ground_station and self.rx_cmd == cmd.TO_MODE_1:
                self.uav.mode = para.MODE_1
                print(self.rx_node + "切换到 MODE_1，等待起飞！")
            elif self.rx_node == self.uav.ground_station and self.rx_cmd == cmd.FLY:
                print("收到起飞指令")
            elif self.rx_cmd == cmd.PREP_SUCCESS:
                self.decision_maker.ready_dict[self.rx_node] = True
                print(self.rx_node + " 准备好了!")
            # elif self.rx_cmd == cmd.REACH_SUCCESS:  # 表示跟踪模式到达指定位置，完成动作
            #     self.decision_maker.ready_dict[self.rx_node] = True
            #     print(self.rx_node + " 已经到达!")
            elif self.rx_cmd == cmd.DETECT_NUM[0]:
                # 更新车的数字
                this_car = self.decision_maker.uav_track_which_car[self.rx_node]  # 都跟1车
                self.decision_maker.real_num = 0

                # First In First Out 调整观测数组
                # self.decision_maker.car_num_detections[this_car].pop(0)  # 去除最早的检测
                # self.decision_maker.car_num_detections[this_car].append(0)  # 加入检测：数字0
                # self.decision_maker.car_num_real[this_car] = 0
                self.decision_maker.new_detection_flag = True
                print("从" + self.rx_node + "收到观测数据" + " car:" + str(this_car) + " number: 0")

            elif self.rx_cmd == cmd.DETECT_NUM[1]:
                # 更新车的数字
                this_car = self.decision_maker.uav_track_which_car[self.rx_node]
                self.decision_maker.real_num = 1

                # First In First Out 调整观测数组
                # self.decision_maker.car_num_detections[this_car].pop(0)  # 去除最早的检测
                # self.decision_maker.car_num_detections[this_car].append(1)  # 加入检测：数字1
                # self.decision_maker.car_num_real[this_car] = 1
                self.decision_maker.new_detection_flag = True
                print("从" + self.rx_node + "收到观测数据" + " car:" + str(this_car) + " number: 1")

            elif self.rx_cmd == cmd.DETECT_NUM[2]:
                # 更新车的数字
                this_car = self.decision_maker.uav_track_which_car[self.rx_node]
                self.decision_maker.real_num = 2

                # First In First Out 调整观测数组
                # self.decision_maker.car_num_detections[this_car].pop(0)  # 去除最早的检测
                # self.decision_maker.car_num_detections[this_car].append(2)  # 加入检测：数字0

                # self.decision_maker.car_num_real[this_car] = 2
                self.decision_maker.new_detection_flag = True
                print("从" + self.rx_node + "收到观测数据" + " car:" + str(this_car) + " number: 2")

            elif self.rx_cmd == cmd.DETECT_NUM[3]:
                # 更新车的数字
                this_car = self.decision_maker.uav_track_which_car[self.rx_node]
                self.decision_maker.real_num = 3

                # First In First Out 调整观测数组
                # self.decision_maker.car_num_detections[this_car].pop(0)  # 去除最早的检测
                # self.decision_maker.car_num_detections[this_car].append(3)  # 加入检测：数字0

                # self.decision_maker.car_num_real[this_car] = 3
                self.decision_maker.new_detection_flag = True
                print("从" + self.rx_node + "收到观测数据" + " car:" + str(this_car) + " number: 3")

            elif self.rx_node == self.uav.ground_station and self.rx_cmd == cmd.RETURN:  # 强制返航
                self.uav.mode = para.MODE_3
                # 发给本机飞控
                self.uav.send_to_fk(cmd.RETURN)

                # 告诉从机该返航了
                while True:
                    try:
                        state = self.uav.read_state_from_fk()
                    except:
                        continue

                    if state is not None:
                        break
                # state = {"latitude": latitude, "longitude": longitude, "z": z, "vx": vx, "vy": vy, "vz": vz, "armed": armed,
                #                     "mode": mode, "yaw": yaw}

                apm_height = 7  # TODO: 设置返航高度
                # id, cmd, type, loc(3), v(3), yaw
                loc = [state['latitude'], state['longitude'], apm_height]
                v = [state['vx'], state['vy'], state['vz']]
                yaw = state['yaw']

                self.data_2_fk.update(id=0, cmd=Follower_Action_RTL, type=TYPE_CLOSE, current_loc=loc, v=v, yaw=yaw)
                data_2_send = self.data_2_fk.trans_data()
                self.data_2_fk.send_broadcast(self.PORT2FK, data_to_send=data_2_send)

                print("强制切换到 MODE_3：返航模式")
                self.uav.send_data_to_one(para.GROUND_STATION_NAME, cmd.RETURN)

                while input("输入'qqqq'以推出程序") == 'qqqq':
                    break

                sys.exit(0)  # 直接结束

        elif self.uav.name == para.GROUND_STATION_NAME:  # 地面站节点
            if self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.ALL_SUCCESS:
                print("全部准备完毕，等待起飞！\n\n")
            elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.RETURN:
                print("开始返航！\n\n")

        else:  # 其他节点
            if self.rx_node == self.uav.ground_station and self.rx_cmd == cmd.TO_MODE_1:
                self.uav.mode = para.MODE_1
                print(self.rx_node + "切换到 MODE_1，等待起飞！")
            elif self.rx_node == self.uav.ground_station and self.rx_cmd == cmd.FLY:
                print("收到起飞指令")
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.TARGET_POS:
            #     self.uav.target_pos = self.rx_position
            #     self.uav.set_init_time()  # 起飞后接到的第一个指令，重置初始化时间
            #     print(self.rx_node + "知道去哪儿了！")
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.TRIM_POS:
            #     self.uav.target_pos = self.rx_position
            #     print(self.rx_node + "知道到哪微调了！")
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.FORMATION_POS:
            #     self.uav.uav_center_position = self.rx_position
            #     print(self.rx_node + "收到中心节点的位置！")

            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.GO_UP:
            #     self.uav.action = cmd.GO_UP
            #     print(self.uav.name + " 正在 " + self.uav.action)
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.GO_DOWN:
            #     self.uav.action = cmd.GO_DOWN
            #     print(self.uav.name + " 正在 " + self.uav.action)
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.GO_TO_CAR_NUM[0]:
            #     self.uav.action = cmd.GO_TO_CAR_NUM[0]
            #     print(self.uav.name + " 正在 " + self.uav.action)
            #     self.uav.send_to_fk(self.uav.action)  # 让fk动作，等待fk飞到
            #     time.sleep(0.5)  # 不能太快，否则fk还没来得及把自身状态更改为False
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.GO_TO_CAR_NUM[1]:
            #     self.uav.action = cmd.GO_TO_CAR_NUM[1]
            #     print(self.uav.name + " 正在 " + self.uav.action)
            #     self.uav.send_to_fk(self.uav.action)  # 让fk动作，等待fk飞到
            #     time.sleep(0.5)  # 不能太快，否则fk还没来得及把自身状态更改为False
            # elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.GO_TO_CAR_NUM[2]:
            #     self.uav.action = cmd.GO_TO_CAR_NUM[2]
            #     print(self.uav.name + " 正在 " + self.uav.action)
            #     self.uav.send_to_fk(self.uav.action)  # 让fk动作，等待fk飞到
            #     time.sleep(0.5)  # 不能太快，否则fk还没来得及把自身状态更改为False

            elif self.rx_node == self.uav.uav_center and self.rx_cmd == cmd.TO_MODE_3:
                self.uav.mode = para.MODE_3
                print(self.rx_node + "切换到 MODE_3")

        self.break_flag = True  # 用于告诉外界去切出循环
