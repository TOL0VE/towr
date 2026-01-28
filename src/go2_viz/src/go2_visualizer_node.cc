#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/init.h>

// 引入 XPP 的核心消息定义
#include <xpp_msgs/topic_names.h> 

// 引入可视化相关的头文件 (转换器 + 可视化器)
#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

// 引入你的 Go1 IK 头文件
#include "go2_inverse_kinematics.h"

using namespace xpp;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "go2_visualizer_node");

  // 1. 定义中间话题名称 (Converter 发给 Visualizer 的暗号)
  const std::string joint_desired_go2 = "xpp/joint_go2_des";

  // =============================================================
  // 第一部分：转换器 (Cartesian -> Joint)
  // =============================================================
  
  // 实例化你的 Go2 IK 求解器
  auto go2_ik = std::make_shared<Go1InverseKinematics>();

  // 创建转换器
  // 参数1: IK 求解器
  // 参数2: 输入话题 (TOWR 发布的笛卡尔轨迹)
  // 参数3: 输出话题 (计算出的关节角度)
  CartesianJointConverter inv_kin_converter(
      go2_ik, 
      xpp_msgs::robot_state_desired, // 默认为 "/xpp/state_des"
      joint_desired_go2
  );

  // =============================================================
  // 第二部分：可视化器 (Joint -> Rviz TF)
  // =============================================================

  // 定义关节名称列表 (顺序必须与 IK 解算顺序一致: LF, RF, LH, RH)
  // 根据你提供的 go2.urdf，名字如下：
  std::vector<std::string> joint_names = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",  // Leg 0: LF
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",  // Leg 1: RF
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",  // Leg 2: LH
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"   // Leg 3: RH
  };

  // 定义参数名称和坐标系
  std::string urdf_param = "go2_rviz_urdf_robot_description";
  std::string base_link = "base"; // go2.urdf 里基座叫 "base"
  std::string world_frame = "world";
  std::string tf_prefix = "go2_des"; // 给 TF 加个前缀，防止冲突

  // 创建可视化器
  // 它会订阅 joint_desired_go2 并在 Rviz 里画图
  UrdfVisualizer go2_desired(
      urdf_param, 
      joint_names, 
      base_link, 
      world_frame,
      joint_desired_go2, 
      tf_prefix
  );

  std::cout << "Go1 Visualizer (Converter + Publisher) is running..." << std::endl;

  // 开始循环 (Converter 和 Visualizer 都在这里面通过回调工作)
  ::ros::spin();

  return 1;
}