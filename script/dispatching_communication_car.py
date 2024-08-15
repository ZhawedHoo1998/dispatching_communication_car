#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import socket
import threading
import time
import rospy 
import ast
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

# global vibe
client_flag = 0
tf_data_str = ""
# 设置坐标系
base_frame_ = '/base_link'
parent_frame_ = '/map'
# tf
tf_buffer = tf2_ros.Buffer()

def on_shutdown():
    rospy.loginfo("程序已停止")
  # stop()
# service handle   true 开始导航 fasle关闭导航


def thread_spin():
    rospy.spin()
    rospy.loginfo_once("spin已启动，循环等待回调函数！")

def get_tf_pose():
    global tf_buffer
    try:
        if not tf_buffer.can_transform('base_link', 'map', rospy.Time()):
            rospy.logerr("无法获取 /map 到 /base_link 的坐标变换")
            return None 
        else:
            trans = tf_buffer.lookup_transform('base_link', 'map', rospy.Time())
            return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("无法获取坐标变换: %s", str(e))
        raise e

def format_tf_data(trans):
    position_str = "position: x={:.2f}, y={:.2f}, z={:.2f}".format(
        trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z
    )
    orientation_str = "orientation: x={:.2f}, y={:.2f}, z={:.2f}, w={:.2f}".format(
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    )
    return "{{{}, {}}}".format(position_str, orientation_str)

def parse_client_data(data_str):
    # 解析客户端发送的字符串数据
    try:
        data = ast.literal_eval(data_str)

        if isinstance(data, dict):
            mission_state = data['mission_state']
            goal_data = data['goal']
            # 解析目标点信息
            target_pose = PoseStamped()
            target_pose.pose.position.x = goal_data[0]
            target_pose.pose.position.y = goal_data[1]
            target_pose.pose.position.z = goal_data[2]
            return mission_state, target_pose
        else:
            rospy.logerr("客户端发送的数据不是字典格式")
            return None, None
        
    except (ValueError, KeyError):
        rospy.logerr("无法解析客户端发送的数据")
        return None, None


def send_tf_data(new_client_socket):
    rospy.loginfo("开始发送车辆位置")
    while not rospy.is_shutdown():
        trans = None
        trans = get_tf_pose()

        if trans is not None:
            tf_data_str = format_tf_data(trans)
            try:
                new_client_socket.sendall(tf_data_str.encode('utf-8'))
                print("发送 tf_data_str 成功")
                # rate.sleep()
            except Exception as e:
                rospy.logerr("发送 tf_data_str 失败: %s", str(e))
                continue
            # rospy.loginfo("车辆位置: %s", tf_data_str)

        rospy.sleep(0.5)  # 每0.5秒发送一次tf数据
        


def client(soc):
    global client_flag
    global tf_data_str
    # define ros publisher
    goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
    mission_state_pub = rospy.Publisher('/mission_state', String, queue_size=10)

    # TODO: 通过tcp发布车辆位置
    # TODO: 发布车辆目标点
    # TODO: 分布任务暂停状态

    try:
        new_client_socket, client_addr = soc.accept()  # 当服务器得到客户端请求连接时，client_flag=1  阻塞式
        client_flag = 1


        rospy.loginfo("客户端连接成功 client_addr = %s",client_addr)

        # 定时发送 tf_data_str 数据给客户端
        rate = rospy.Rate(0.2)  # 1 Hz

        tf_thread = threading.Thread(target=send_tf_data, args=(new_client_socket,))
        tf_thread.start()

        while not rospy.is_shutdown():
            
            # 接收客户端发送的数据
            recv_data = new_client_socket.recv(1024).decode('utf-8')
            print("recv_data: ", type(recv_data))


            if recv_data:
                # rospy.loginfo("收到客户端数据：%s", recv_data)
                # TODO: 根据接收到的数据，发布车辆目标点
                # recv_data ="{'goal': [0, 0, 0], 'mission_state': 'go'}"

                mission_state, goal = parse_client_data(recv_data)
                if goal!=None:
                    goal.header.stamp = rospy.Time.now()
                    goal_pub.publish(goal)
                    rospy.loginfo("发布目标点：%s", goal)
                else:
                    rospy.loginfo("未收到目标点")
                
                if mission_state != None:
                    mission_state_pub.publish(mission_state)
                    rospy.loginfo("发布任务状态：%s", mission_state)
                else:
                    rospy.loginfo("未收到任务状态")

            else:
                rospy.loginfo("客户端断开连接")
                break
    except Exception as e:
        rospy.logerr("客户端处理失败: %s", str(e))
        
    finally:
        try:
            if new_client_socket is not None:
                new_client_socket.shutdown(socket.SHUT_RDWR)
                new_client_socket.close()
        except:
            rospy.logerr("无法关闭客户端socket")
        client_flag = 0

    

def main():
    global client_flag
    global tf_buffer
    #  创建socket
    rospy.init_node('dispatching_communication_car')
    rospy.on_shutdown(on_shutdown) #退出回调函数 停车
    rospy.loginfo("启动调度系统与小车中间件！")
    
    server_ip_ = rospy.get_param('server_ip')
    server_port_ = rospy.get_param('server_port')
    rospy.loginfo("server_ip_=%s server_port_=%d",server_ip_,server_port_)

    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #  监听端口
    #  soc.bind(('192.168.43.141', 8087))
    soc.bind((server_ip_, server_port_))
    # listen()方法开始监听端口，传入的参数指定等待连接的最大数量
    soc.listen(1)
    rospy.loginfo("等待一个新客户端连接....")
    
    client1_threading = threading.Thread(target=client, args=(soc,))
    client1_threading.start()

    # spin_threading = threading.Thread(target=thread_spin)
    # spin_threading.start()

    # tf listener
    
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(60.0)

    while not rospy.is_shutdown():
        try:
            if client_flag:  # 当client_flag为1时，即服务器得到客户端请求连接时，开始一个新的线程
                client1_threading = threading.Thread(target=client, args=(soc,))  # 新建一个线程
                client1_threading.start()  # 开启这个线程
                client_flag = 0  # 标志为，目的使线程不会一直增多，只有当服务器得到客户端请求连接时，才开始一个新的线程
            # 
            
            rate.sleep()
        except:
            print("failer!!")


if __name__ == '__main__':
  main()
