#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys
import os

def play_csv(csv_path):
    rospy.init_node('go1_csv_player')
    
    # 1. 设置 Publisher
    # 发布关节角度
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # TF 广播器 (发布基座移动)
    tf_broadcaster = tf.TransformBroadcaster()

    # 2. 读取 CSV
    if not os.path.exists(csv_path):
        rospy.logerr("找不到文件: " + csv_path)
        return

    rospy.loginfo("正在加载 CSV: " + csv_path)
    df = pd.read_csv(csv_path, index_col=False)
    
    rospy.loginfo("加载完成，共 %d 帧数据。3秒后开始播放...", len(df))
    rospy.sleep(3.0)

    # 3. 定义关节名称 (必须和 URDF 以及你的 IK 顺序完全一致)
    # 顺序: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
    joint_names = [
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",   # LF
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",   # RF
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",   # LH
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"    # RH
    ]

    rate = rospy.Rate(50) # 50Hz (对应 dt=0.02)

    # 4. 循环播放
    # while not rospy.is_shutdown(): # 如果想循环播放就把这行注释打开
    for index, row in df.iterrows():
        if rospy.is_shutdown(): break

        current_time = rospy.Time.now()

        # --- A. 发布基座 TF (World -> Base) ---
        # 对应 CSV 列: base_x, base_y, base_z, base_quat_x...
        base_pos = (row['base_x'], row['base_y'], row['base_z'])
        # 原始四元数
        raw_quat = [
            row['base_quat_x'], 
            row['base_quat_y'], 
            row['base_quat_z'], 
            row['base_quat_w']
        ]

        # =========== 【新增：强制归一化】 ===========
        import math
        # 计算模长
        norm = math.sqrt(sum(x*x for x in raw_quat))
        
        # 如果模长有效，则归一化；否则给一个默认值
        if norm > 1e-6:
            base_quat = [x / norm for x in raw_quat]
        else:
            base_quat = [0, 0, 0, 1] # 默认单位四元数
        # ==========================================
        
        # 广播 TF: world -> base
        tf_broadcaster.sendTransform(
            base_pos,
            base_quat,
            current_time,
            "base",  # 子坐标系 (你的 URDF 基座名字)
            "world"  # 父坐标系 (固定世界坐标)
        )

        # --- B. 发布关节角度 (Joint States) ---
        # 对应 CSV 列: q0 ~ q11
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = current_time
        msg.name = joint_names
        
        # 提取 q0 到 q11
        q_values = [
            row['q0'], row['q1'], row['q2'],
            row['q3'], row['q4'], row['q5'],
            row['q6'], row['q7'], row['q8'],
            row['q9'], row['q10'], row['q11']
        ]
        msg.position = q_values
        
        # (可选) 也可以发布速度
        # dq_values = [row['dq0'], ... ]
        # msg.velocity = dq_values

        joint_pub.publish(msg)

        # 控制播放速度
        rate.sleep()

        if index % 50 == 0:
            rospy.loginfo("播放进度: %.2f / %.2f 秒", row['time'], df.iloc[-1]['time'])

    rospy.loginfo("播放结束！")

if __name__ == '__main__':
    default_path = "/home/noix/towr_project/src/go1_viz/data/go1_rl_data.csv"
    
    # 支持命令行参数传入路径
    path = sys.argv[1] if len(sys.argv) > 1 else default_path
    
    try:
        play_csv(path)
    except rospy.ROSInterruptException:
        pass