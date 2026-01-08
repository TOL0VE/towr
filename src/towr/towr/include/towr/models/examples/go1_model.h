#ifndef TOWR_MODELS_GO1_MODEL_H_
#define TOWR_MODELS_GO1_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>

namespace towr {

// ==========================================
// 1. 定义 Go1 的运动学参数 (腿长与活动范围)
// ==========================================
class Go1KinematicModel : public KinematicModel {
public:
  Go1KinematicModel () : KinematicModel(4) // 4条腿
  {
    // === 参数来源：go1.urdf ===
    // 髋关节 X 偏移: 0.1881
    // 髋关节 Y 偏移: 0.04675 + 0.08 (侧展臂长) ≈ 0.13
    // 站立高度 Z: 大腿 0.213 + 小腿 0.213，站立时稍微弯曲，设为 -0.3m 左右比较合理
    const double x_nom = 0.1881;
    const double y_nom = 0.13;
    const double z_nom = -0.30; // 标称站立高度

    // 定义四条腿相对于基座中心的名义足端位置
    // 顺序通常是: LF, RF, LH, RH
    nominal_stance_.at(0) <<  x_nom,  y_nom, z_nom;
    nominal_stance_.at(1) <<  x_nom, -y_nom, z_nom;
    nominal_stance_.at(2) << -x_nom,  y_nom, z_nom;
    nominal_stance_.at(3) << -x_nom, -y_nom, z_nom;

    // 定义每条腿的活动范围 (Workspace)
    // 这是一个长方体盒子，用来约束求解器不要把腿伸得太远
    max_dev_from_nominal_ << 0.15, 0.1, 0.10; // X, Y, Z 方向的活动半径
  }
};

// ==========================================
// 2. 定义 Go1 的动力学参数 (质量与惯性)
// ==========================================
class Go1DynamicModel : public SingleRigidBodyDynamics {
public:
  Go1DynamicModel()
  : SingleRigidBodyDynamics(12.0, // 质量 (Mass)
                    0.02, 0.07, 0.08, // Ixx, Iyy, Izz
                    0.0, 0.0, 0.0,    // Ixy, Ixz, Iyz
                    4)                // 4条腿
  {}
};

} /* namespace towr */

#endif /* TOWR_MODELS_GO1_MODEL_H_ */