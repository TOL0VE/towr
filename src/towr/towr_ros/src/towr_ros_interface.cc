/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_msgs/TerrainInfo.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/convert.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

#include <fstream>
#include <ros/package.h>
#include <sys/stat.h>

#include <visualization_msgs/Marker.h>

namespace towr {

TowrRosInterface::TowrRosInterface() {
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_pub_ = n.advertise<xpp_msgs::RobotStateCartesian>(
      xpp_msgs::robot_state_desired, 1);

  robot_parameters_pub_ =
      n.advertise<xpp_msgs::RobotParameters>(xpp_msgs::robot_parameters, 1);

  solver_ = std::make_shared<ifopt::IpoptSolver>();

  visualization_dt_ = 0.01;
}

BaseState TowrRosInterface::GetGoalState(const TowrCommandMsg &msg) const {
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  return goal;
}

Eigen::VectorXd Go1_SolveSingleLeg_Embedded(const Eigen::Vector3d &p,
                                            int leg_id) {
    Eigen::VectorXd q(3);

    // ========================================================
    // 【修改点】：根据 Go2 URDF 更新参数
    // ========================================================
    const double l0 = 0.0955; // 原为 0.08，Go2 实测为 0.0955 
    const double l1 = 0.213;  // Go2 大腿长度 
    const double l2 = 0.213;  // Go2 小腿长度 

    bool is_right = (leg_id == 1 || leg_id == 3);

    // ========================================================
    // 1. 侧展 q[0] (保持不变)
    // ========================================================
    double l_yz = std::sqrt(p.y()*p.y() + p.z()*p.z());
    double lyz_ratio = l0 / l_yz;
    
    // 保护：防止 asin 输入越界
    if (lyz_ratio > 1.0) lyz_ratio = 1.0;
    if (lyz_ratio < -1.0) lyz_ratio = -1.0;

    double q0_geom = std::asin(lyz_ratio) + std::atan2(std::abs(p.z()), std::abs(p.y()));
    q[0] = q0_geom - 1.57079632679; // -90度偏移

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

    // 【保持您的修改】：Go2 膝盖通常向后弯曲 (Config不同可能定义不同，保持您的负值逻辑)
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

std::vector<double>
Go1_GetAllJoints(const std::vector<Eigen::Vector3d> &feet_pos) {
  std::vector<double> q_all;
  // Go1 髋关节偏移 (LF, RF, LH, RH)
  double hx = 0.1881;
  double hy = 0.04675;
  std::vector<Eigen::Vector3d> hip_offsets = {
      Eigen::Vector3d(hx, hy, 0.0), Eigen::Vector3d(hx, -hy, 0.0),
      Eigen::Vector3d(-hx, hy, 0.0), Eigen::Vector3d(-hx, -hy, 0.0)};

  for (int i = 0; i < 4; ++i) {
    Eigen::Vector3d p_H = feet_pos[i] - hip_offsets[i];
    Eigen::VectorXd q = Go1_SolveSingleLeg_Embedded(p_H, i);
    q_all.push_back(q[0]);
    q_all.push_back(q[1]);
    q_all.push_back(q[2]);
  }
  return q_all;
}

void TowrRosInterface::UserCommandCallback(const TowrCommandMsg &msg) {
  // robot model
  formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
  auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // terrain
  auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
  formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

  int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation_.params_ = GetTowrParameters(n_ee, msg);
  formulation_.final_base_ = GetGoalState(msg);

  SetTowrInitialState();

  // solver parameters
  SetIpoptParameters(msg);

  // visualization
  PublishInitialState();

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize || msg.play_initialization) {
    nlp_ = ifopt::Problem();
    for (auto c : formulation_.GetVariableSets(solution))
      nlp_.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
      nlp_.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
      nlp_.AddCostSet(c);

    solver_->Solve(nlp_);
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    int success =
        system(("rosbag play --topics " + xpp_msgs::robot_state_desired + " " +
                xpp_msgs::terrain_info + " -r " +
                std::to_string(msg.replay_speed) + " --quiet " + bag_file)
                   .c_str());
  }

  if (msg.replay_trajectory) {

    if (!solution.base_linear_) {
      ROS_WARN("⚠️ solution is empty. Optimize first!");
      return;
    }

    // 1. 设置路径与高精度
    std::string path = "/home/o/towr/data/go2_motion.csv";
    // mkdir("/home/o/towr/data", 0777); // 如果目录不存在需解注
    std::ofstream file(path);
    if (!file.is_open()) { ROS_ERROR("Can't open file"); return; }
    
    ROS_INFO("💾 Exporting HIGH-PRECISION CSV to: %s", path.c_str());

    // 🔥 关键优化：提高浮点数精度，防止抖动
    file << std::fixed << std::setprecision(9);

    // 写入 Header (对齐 Isaac Lab 习惯: Quaternion W 在前)
    file << "time,"
        << "base_x,base_y,base_z,"
        << "base_quat_w,base_quat_x,base_quat_y,base_quat_z," // 注意顺序 WXYZ
        << "base_lin_x,base_lin_y,base_lin_z,"
        << "base_ang_x,base_ang_y,base_ang_z,"
        << "q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,"
        << "dq0,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,dq9,dq10,dq11\n"; 

    // 🔥 关键优化：使用 60Hz (0.0166s) 匹配 Isaac Lab 默认控制频率
    double dt = 1.0 / 60.0; 
    double T = solution.base_linear_->GetTotalTime();
    
    EulerConverter base_angular(solution.base_angular_);
    
    // 髋关节偏移 (Go1)
    double hx = 0.1881, hy = 0.04675;
    std::vector<Eigen::Vector3d> hip_offsets = {
        {hx, hy, 0.0}, {hx, -hy, 0.0}, {-hx, hy, 0.0}, {-hx, -hy, 0.0}};

    // 2. 预计算所有点 (为了使用中心差分计算平滑速度)
    struct FrameData {
      double t;
      Eigen::Vector3d p_W, v_W, w_W;
      Eigen::Quaterniond q_W;
      std::vector<double> joint_q;
    };
    std::vector<FrameData> trajectory;

    // 循环采样所有位置点
    for (double t = 0.0; t <= T + 1e-5; t += dt) {
      FrameData frame;
      frame.t = t;
      frame.p_W = solution.base_linear_->GetPoint(t).p();
      frame.v_W = solution.base_linear_->GetPoint(t).v();
      
      // 姿态
      frame.q_W = base_angular.GetQuaternionBaseToWorld(t);
      frame.w_W = base_angular.GetAngularVelocityInWorld(t);
      
      // IK 解算关节角度 (复用你现有的逻辑)
      Eigen::Matrix3d R_BW = frame.q_W.toRotationMatrix().transpose();
      frame.joint_q.resize(12);
      
      int n_ee = solution.ee_motion_.size();
      for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
        int ee_idx = ToXppEndeffector(n_ee, ee_towr).first;
        Eigen::Vector3d feet_pos_W = solution.ee_motion_.at(ee_towr)->GetPoint(t).p();
        Eigen::Vector3d feet_pos_H = R_BW * (feet_pos_W - frame.p_W) - hip_offsets[ee_idx];
        Eigen::VectorXd q_leg = Go1_SolveSingleLeg_Embedded(feet_pos_H, ee_idx);
        
        frame.joint_q[3 * ee_idx + 0] = q_leg[0];
        frame.joint_q[3 * ee_idx + 1] = q_leg[1];
        frame.joint_q[3 * ee_idx + 2] = q_leg[2];
      }
      trajectory.push_back(frame);
    }

    // 3. 写入文件 & 计算关节速度 (中心差分 Central Difference)
    for (size_t i = 0; i < trajectory.size(); ++i) {
      const auto& f = trajectory[i];
      
      // 计算 dq (Joint Velocity)
      std::vector<double> dq(12, 0.0);
      if (i > 0 && i < trajectory.size() - 1) {
        // 中间点：中心差分 (q[t+1] - q[t-1]) / 2dt -> 最平滑
        for(int j=0; j<12; ++j) 
          dq[j] = (trajectory[i+1].joint_q[j] - trajectory[i-1].joint_q[j]) / (2.0 * dt);
      } else if (i == 0 && trajectory.size() > 1) {
        // 起点：前向差分
        for(int j=0; j<12; ++j) 
          dq[j] = (trajectory[i+1].joint_q[j] - f.joint_q[j]) / dt;
      } else if (i == trajectory.size() - 1 && i > 0) {
        // 终点：后向差分
        for(int j=0; j<12; ++j) 
          dq[j] = (f.joint_q[j] - trajectory[i-1].joint_q[j]) / dt;
      }

      // 写入 CSV
      file << f.t << ","
          << f.p_W.x() << "," << f.p_W.y() << "," << f.p_W.z() << ","
          << f.q_W.w() << "," << f.q_W.x() << "," << f.q_W.y() << "," << f.q_W.z() << ","
          << f.v_W.x() << "," << f.v_W.y() << "," << f.v_W.z() << ","
          << f.w_W.x() << "," << f.w_W.y() << "," << f.w_W.z() << ",";
      
      for (double v : f.joint_q) file << v << ",";
      for (size_t j = 0; j < 12; ++j) file << dq[j] << (j == 11 ? "" : ","); 
      
      file << "\n";
    }

    file.close();
    ROS_INFO("✅ CSV Export Done! Frames: %lu, DT: %.4f", trajectory.size(), dt);
  }

  if (msg.plot_trajectory) {
    int success =
        system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // to publish entire trajectory (e.g. to send to controller)
  // xpp_msgs::RobotStateCartesianTrajectory xpp_msg =
  // xpp::Convert::ToRos(GetTrajectory());
}

void TowrRosInterface::PublishInitialState() {
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q = EulerConverter::GetQuaternionBaseToWorld(
      formulation_.initial_base_.ang.p());

  for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp) = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions() {
  std::vector<XppVec> trajectories;

  for (int iter = 0; iter < nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec TowrRosInterface::GetTrajectory() const {
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t <= T + 1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) =
          solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp) =
          ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_.at(ee_xpp) =
          solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel &model) const {
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev =
      xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr = 0; ee_towr < n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(
        xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void TowrRosInterface::SaveOptimizationAsRosbag(
    const std::string &bag_name, const xpp_msgs::RobotParameters &robot_params,
    const TowrCommandMsg user_command_msg, bool include_iterations) {
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command + "_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i = 0; i < n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i),
                             towr_msgs::nlp_iterations_name +
                                 std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void TowrRosInterface::SaveTrajectoryInRosbag(rosbag::Bag &bag,
                                              const XppVec &traj,
                                              const std::string &topic) const {
  for (const auto state : traj) {
    auto timestamp =
        ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = formulation_.terrain_->GetNormalizedBasis(
          HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(
          xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */
