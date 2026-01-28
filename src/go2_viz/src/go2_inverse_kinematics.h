#ifndef GO2_INVERSE_KINEMATICS_H
#define GO2_INVERSE_KINEMATICS_H

#include <xpp_vis/inverse_kinematics.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
class Go2InverseKinematics : public xpp::InverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;

  Go2InverseKinematics() = default;
  virtual ~Go2InverseKinematics() = default;

  int GetEECount() const override { return 4; }

  xpp::Joints GetAllJointAngles(const xpp::EndeffectorsPos& x_B) const override 
  {
    auto pos_B = x_B.ToImpl(); 
    if (pos_B.size() != 4) pos_B.resize(4, Vector3d::Zero());
    // ==========================================
    // 🔴 DEBUG 可视化代码开始
    // ==========================================
    // static ros::NodeHandle nh;
    // static ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("ik_target_debug", 1);

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "go1_des/base"; // 注意：这里假设 pos_B 是相对于 base 的
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "debug_points";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::SPHERE_LIST; // 画球列表
    // marker.action = visualization_msgs::Marker::ADD;
    
    // // 设置球的大小 (0.05米 = 5厘米)
    // marker.scale.x = 0.10;
    // marker.scale.y = 0.10;
    // marker.scale.z = 0.10;
    
    // // 设置颜色：红色 (Red)
    // marker.color.r = 1.0f;
    // marker.color.g = 0.0f;
    // marker.color.b = 0.0f;
    // marker.color.a = 1.0f; // 不透明

    // // 把 IK 接收到的点填进去
    // for (const auto& p : pos_B) {
    //     geometry_msgs::Point pt;
    //     pt.x = p.x();
    //     pt.y = p.y();
    //     pt.z = p.z();
    //     marker.points.push_back(pt);
    // }

    // // 发布！
    // debug_pub.publish(marker);
    // ==========================================
    // 🔴 DEBUG 可视化代码结束
    // ==========================================
    std::vector<Eigen::VectorXd> q_vec;

    // Go2 髋关节偏移 (Base -> Hip)
    double hx = 0.1881;
    double hy = 0.04675;
    
    std::vector<Vector3d> hip_offsets = {
        Vector3d( hx,  hy, 0.0), // LF
        Vector3d( hx, -hy, 0.0), // RF
        Vector3d(-hx,  hy, 0.0), // LH
        Vector3d(-hx, -hy, 0.0)  // RH
    };

    for (int ee = 0; ee < 4; ++ee) {
      // 1. 转换到髋关节坐标系
      Vector3d p_H = pos_B.at(ee) - hip_offsets.at(ee);
      
      // 2. 解算
      q_vec.push_back(SolveSingleLeg(p_H, ee));
    }

    // ================= DEBUG 打印开始 =================
    // std::cout << "-------- IK Solution --------" << std::endl;
    // std::vector<std::string> leg_names = {"LF", "RF", "LH", "RH"};
    
    // for (size_t i = 0; i < q_vec.size(); ++i) {
    //     // .transpose() 将列向量转置为行向量打印，看起来像 [q0, q1, q2]
    //     std::cout << leg_names[i] << ": " << q_vec[i].transpose() << std::endl;
    // }
    // std::cout << "-----------------------------" << std::endl;
    // ================= DEBUG 打印结束 =================

    return xpp::Joints(q_vec);
  }

private:
Eigen::VectorXd SolveSingleLeg(const Vector3d& p, int leg_id) const
  {
    Eigen::VectorXd q(3);

    const double l0 = 0.08;
    const double l1 = 0.213;
    const double l2 = 0.213;

    bool is_right = (leg_id == 1 || leg_id == 3);

    // ========================================================
    // 1. 侧展 q[0] (保持不变)
    // ========================================================
    double l_yz = std::sqrt(p.y()*p.y() + p.z()*p.z());
    double lyz_ratio = l0 / l_yz;
    if (lyz_ratio > 1.0) lyz_ratio = 1.0;
    if (lyz_ratio < -1.0) lyz_ratio = -1.0;

    double q0_geom = std::asin(lyz_ratio) + std::atan2(std::abs(p.z()), std::abs(p.y()));
    q[0] = q0_geom - 1.57079632679;

    if (!is_right) q[0] = -q[0];


    // ========================================================
    // 2. 准备计算
    // ========================================================
    double l_proj_sq = l_yz*l_yz - l0*l0;
    if (l_proj_sq < 0) l_proj_sq = 0;
    
    double l_total_sq = p.x()*p.x() + l_proj_sq;
    double l_total = std::sqrt(l_total_sq);

    // ========================================================
    // 3. 膝盖 q[2] 
    // ========================================================
    double cos_q2 = (l1*l1 + l2*l2 - l_total_sq) / (2 * l1 * l2);
    if (cos_q2 > 1.0) cos_q2 = 1.0;
    if (cos_q2 < -1.0) cos_q2 = -1.0;

    // 【修改点 A】：改为负值
    // 之前正值导致了“向前弯”，现在改回负值
    q[2] = - (M_PI - std::acos(cos_q2)); 


    // ========================================================
    // 4. 大腿 q[1] 
    // ========================================================
    double beta = std::acos((l_total_sq + l1*l1 - l2*l2) / (2 * l_total * l1));
    if (std::isnan(beta)) beta = 0.0;

    double l_proj_corrected = std::sqrt(l_proj_sq);
    double phi = std::atan2(p.x(), l_proj_corrected);


    q[1] = -phi + beta;

    return q;
  }
};

#endif