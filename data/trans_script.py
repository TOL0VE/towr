import pandas as pd
import numpy as np
import pinocchio as pin
import os
import glob
import tqdm  # 建议安装 tqdm 显示进度条: pip install tqdm

# ==================== ⚙️ 配置区域 ====================
URDF_FILENAME = "go1.urdf"  # 确保 URDF 文件在同级目录
# 关节名称 (必须严格对应 URDF)
DOF_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
]
# 需要导出的 Body 名称 (用于 AMP 观测)
BODY_NAMES = [
    "trunk", 
    "FL_hip", "FL_thigh", "FL_calf", "FL_foot",
    "FR_hip", "FR_thigh", "FR_calf", "FR_foot",
    "RL_hip", "RL_thigh", "RL_calf", "RL_foot",
    "RR_hip", "RR_thigh", "RR_calf", "RR_foot"
]
# ====================================================

def convert_single_file(csv_path, model, data, body_indices):
    """处理单个 CSV 文件并返回处理好的数据字典"""
    try:
        df = pd.read_csv(csv_path)
        
        # 0. 基础检查
        required_cols = ["base_x", "q0", "dq0", "base_quat_w"]
        if not all(col in df.columns for col in required_cols):
            print(f"⚠️ 跳过 {csv_path}: 缺少关键列，可能不是运动数据。")
            return None

        # 1. 计算 FPS
        dt = df["time"][1] - df["time"][0]
        fps = 1.0 / dt
        n_frames = len(df)

        # 2. 提取 CSV 原始数据
        # Base Data
        root_pos = df[["base_x", "base_y", "base_z"]].values
        # C++ 导出时已经是 [w, x, y, z]，直接用
        root_rot = df[["base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"]].values
        root_lin_vel = df[["base_lin_x", "base_lin_y", "base_lin_z"]].values
        root_ang_vel = df[["base_ang_x", "base_ang_y", "base_ang_z"]].values
        
        # Joint Data
        dof_cols = [f"q{i}" for i in range(12)]
        dq_cols = [f"dq{i}" for i in range(12)]
        dof_pos = df[dof_cols].values
        dof_vel = df[dq_cols].values

        # 3. 准备输出容器
        all_body_pos = np.zeros((n_frames, len(BODY_NAMES), 3))
        all_body_rot = np.zeros((n_frames, len(BODY_NAMES), 4))
        all_body_lin_vel = np.zeros((n_frames, len(BODY_NAMES), 3))
        all_body_ang_vel = np.zeros((n_frames, len(BODY_NAMES), 3))

        # 4. 逐帧计算 FK
        for i in range(n_frames):
            # Pinocchio 需要的 Quaternion 顺序是 [x, y, z, w]
            # 我们 CSV 里 root_rot 是 [w, x, y, z]，需要调整顺序喂给 Pinocchio
            quat_xyzw = [root_rot[i][1], root_rot[i][2], root_rot[i][3], root_rot[i][0]]
            
            q_frame = np.concatenate([root_pos[i], quat_xyzw, dof_pos[i]])
            v_frame = np.concatenate([root_lin_vel[i], root_ang_vel[i], dof_vel[i]])
            
            # 正运动学计算
            pin.forwardKinematics(model, data, q_frame, v_frame)
            pin.updateFramePlacements(model, data)

            for j, frame_id in enumerate(body_indices):
                # 获取位置
                placement = data.oMf[frame_id]
                all_body_pos[i, j] = placement.translation
                
                # 获取旋转 (Pinocchio [x,y,z,w] -> Isaac [w,x,y,z])
                q_pin = pin.Quaternion(placement.rotation)
                all_body_rot[i, j] = [q_pin.w, q_pin.x, q_pin.y, q_pin.z]
                
                # 获取速度
                vel = pin.getFrameVelocity(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
                all_body_lin_vel[i, j] = vel.linear
                all_body_ang_vel[i, j] = vel.angular

        return {
            "fps": fps,
            "dof_names": np.array(DOF_NAMES),
            "body_names": np.array(BODY_NAMES),
            "dof_positions": dof_pos,
            "dof_velocities": dof_vel,
            "body_positions": all_body_pos,
            "body_rotations": all_body_rot,
            "body_linear_velocities": all_body_lin_vel,
            "body_angular_velocities": all_body_ang_vel
        }

    except Exception as e:
        print(f"❌ 处理 {csv_path} 时出错: {e}")
        return None

def main():
    # 1. 检查 URDF
    if not os.path.exists(URDF_FILENAME):
        print(f"❌ 找不到 URDF 文件: {URDF_FILENAME}")
        print("请将 urdf 文件放在脚本同级目录下。")
        return

    # 2. 初始化 Pinocchio 模型 (只做一次)
    print(f"🤖 正在加载模型: {URDF_FILENAME} ...")
    model = pin.buildModelFromUrdf(URDF_FILENAME)
    data = model.createData()
    
    # 缓存 Body ID
    body_indices = []
    for name in BODY_NAMES:
        if model.existFrame(name):
            body_indices.append(model.getFrameId(name))
        elif name == "trunk" and model.existFrame("base"): # 兼容 trunk/base 命名
            body_indices.append(model.getFrameId("base"))
        else:
            print(f"❌ 致命错误: URDF 中找不到 Body: {name}")
            return

    # 3. 扫描 CSV 文件
    csv_files = glob.glob("*.csv")
    if not csv_files:
        print("📂 当前目录下没有找到 .csv 文件。")
        return

    print(f"📂 找到 {len(csv_files)} 个 CSV 文件，准备开始转换...")

    # 4. 批量转换
    success_count = 0
    # 使用 tqdm 显示进度条，如果没有安装 tqdm，直接用 csv_files 循环
    iterator = tqdm.tqdm(csv_files) if 'tqdm' in globals() else csv_files
    
    for csv_file in iterator:
        npz_filename = os.path.splitext(csv_file)[0] + ".npz"
        
        # 转换数据
        result = convert_single_file(csv_file, model, data, body_indices)
        
        if result:
            # 保存
            np.savez(npz_filename, **result)
            if 'tqdm' not in globals():
                print(f"✅ 已转换: {csv_file} -> {npz_filename}")
            success_count += 1

    print("\n" + "="*40)
    print(f"🎉 任务完成！")
    print(f"成功转换: {success_count} / {len(csv_files)}")
    print("="*40)

if __name__ == "__main__":
    main()