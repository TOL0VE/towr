#ifndef TOWR_MODELS_GO2_MODEL_H_
#define TOWR_MODELS_GO2_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>

namespace towr {

// ==========================================
// 1. 定义 Go2 的运动学参数
// ==========================================
class Go2KinematicModel : public KinematicModel {
public:
  Go2KinematicModel () : KinematicModel(4) // 4条腿
  {
    // === 参数来源：go2_description.urdf ===
    // 1. 髋关节 X 偏移 (FL_hip_joint): 0.1934
    // 2. 髋关节 Y 偏移 (FL_hip_joint): 0.0465
    // 3. 侧展臂长 (FL_thigh_joint Y): 0.0955
    // -> 总 Y 偏移 = 0.0465 + 0.0955 = 0.142
    
    const double x_nom = 0.1934;
    const double y_nom = 0.142;
    const double z_nom = -0.30; // 标称站立高度 (大腿0.213+小腿0.213)

    // 定义四条腿相对于基座中心的名义足端位置
    // 顺序: LF, RF, LH, RH
    nominal_stance_.at(0) <<  x_nom,  y_nom, z_nom;
    nominal_stance_.at(1) <<  x_nom, -y_nom, z_nom;
    nominal_stance_.at(2) << -x_nom,  y_nom, z_nom;
    nominal_stance_.at(3) << -x_nom, -y_nom, z_nom;

    // 定义每条腿的活动范围 (Workspace)
    max_dev_from_nominal_ << 0.15, 0.1, 0.12; // 稍微增加Z轴范围，Go2腿较长
  }
};

// ==========================================
// 2. 定义 Go2 的动力学参数
// ==========================================
class Go2DynamicModel : public SingleRigidBodyDynamics {
public:
  Go2DynamicModel()
  // 质量估算 (根据 URDF):
  // Base: ~6.92 kg
  // 单腿总重: ~2.29 kg (Hip:0.68+0.09, Thigh:1.15+0.09, Calf:0.15+0.09, Foot:0.04)
  // 总重: 6.92 + 4 * 2.29 ≈ 16.08 kg
  : SingleRigidBodyDynamics(16.08, 
                    // 惯性张量 (来自 URDF <link name="base">)
                    0.02448, 0.098077, 0.107, // Ixx, Iyy, Izz
                    0.00012, 0.00148, -0.00003, // Ixy, Ixz, Iyz
                    4)                // 4条腿
  {}
};

} /* namespace towr */

#endif /* TOWR_MODELS_GO2_MODEL_H_ */