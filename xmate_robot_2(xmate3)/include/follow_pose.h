#pragma once

#include <iostream>
#include <array>
#include <cmath>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mutex>
#include "robot.h"
#include "model.h"

class ApplyVelSixJoints {
public:
  int Init(const std::array<double, 6> &q, const double delta_t=0.001);
  void Apply(const std::array<double, 6> &dq_des,
             std::array<double, 6> &q_cmd,
             std::array<double, 6> &dq_cmd);

private:
  typedef enum{
    FLAGACC, // Axis in acceleration
    FLAGCTE, // Axis at constant velocity
    FLAGDEC, // Axis in deceleration
    FLAGSTO  // Axis stopped
  } status_t;
  size_t njoints_;
  double delta_t_;

  std::array<double, 6> q_min_;    // Joint min position
  std::array<double, 6> q_max_;    // Joint max position
  std::array<double, 6> dq_max_;   // Joint max velocity
  std::array<double, 6> ddq_max_;  // Joint max acceleration

  std::array<double, 6> q_final_;       // Final position before joint limit 安全位置限制距离
  std::array<double, 6> q_cmd_;         // Joint position command
  std::array<double, 6> q_cmd_prev_;    // Previous joint position command
  std::array<double, 6> dq_des_;      // Desired velocity  期望关节速度
  std::array<double, 6> dq_des_prev_; // Previous desired velocity  上一次的期望关节速度
  std::array<double, 6> delta_q_;		   // Current position increment 当前周期的位置增量
  std::array<double, 6> delta_q_max_;   // Max position increment  当前周期的最大位置增量
  const double delta_q_min_ =	1e-9;	   // Delta q minimum (rad)
  std::array<int, 6> sign_;	         // Displacement sign: +1 = increment position 速度方向，正向或反向
  std::array<double, 6> dist_to_final_; // Distance between current joint position and final
  std::array<double, 6> dist_ad_;       // Distance required to accelerate or decelerate
  std::array<bool, 6> flag_speed_;    // Change of speed direction  速度的方向是否改变
  std::array<status_t, 6> status_;    // Axis status
  std::array<double, 6> delta_q_acc_;	 // Increment related to acceleration 以最大加速度运行单个周期的距离增量
  bool flag_joint_limit_;  // 是否关节被限制运动
  const double offset_joint_limit_ = 1.0/180.0*M_PI; // stop before joint limit (rad) 在关节限制之前一段距离停止
};

class FollowPoseSixJoints {
public:
  /**
   * 构造函数
   *
   * @param[in] robot，机器人句柄.
   * @param[in] fMe，末端相对于法兰的位姿.
   */
  FollowPoseSixJoints(xmate::Robot& robot, xmate::XmateModel& robot_model,
                      const Eigen::Transform<double, 3, Eigen::Isometry>& fMe = Eigen::Transform<double, 3, Eigen::Isometry>::Identity());
  /**
   * 开始目标跟随
   *
   * @param[in] bMe_desire，期望的目标位姿，即TCP位姿.
   */
  void start(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);
  /**rm
   * 停止目标跟随
   *
   */
  void stop();

  /**
   * 更新期望的目标位姿，即TCP位姿
   *
   */
  void update(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);
private:
  /**
   * 实时回调函数
   *
   */
  xmate::JointPositions control_callback(RCI::robot::RobotState robot_state);

  /**
   * 根据当前位姿和目标位姿，使用插值+PID（当前仅P=0.5），得到笛卡尔空间速度
   *
   * @param[in] pose_cur，当前位姿.
   * @param[in] pose_goal，目标位姿.
   * @param[out] cart_vel，以当前位姿为参考坐标系的笛卡尔空间速度.
   */
  int CalcCartVel(const Eigen::Transform<double, 3, Eigen::Isometry>& pose_cur, const Eigen::Transform<double, 3, Eigen::Isometry>& pose_goal,
                Eigen::Matrix<double, 6, 1>& cart_vel);
  /**
   * 计算当前位姿和目标位姿之间的距离
   *
   * @param[in] pose_cur，当前位姿.
   * @param[in] pose_goal，目标位姿.
   * @param[out] dis，距离.
   */
  int PosePairDis(const Eigen::Transform<double, 3, Eigen::Isometry>& pose_cur, const Eigen::Transform<double, 3, Eigen::Isometry>& pose_goal,
                  double& dis);
  /**
   * 转换雅可比矩阵的笛卡尔空间速度相对坐标系
   *
   * @param[in] bJf，雅可比矩阵，输入关节角速度dq，输出末端相对于基坐标系的速度bVf.
   * @param[in] bMe，末端相对于基坐标系的位姿.
   * @param[out] fJf，雅可比矩阵，输入关节角速度dq，输出末端相对于当前末端坐标系的速度fVf.
   */
  void bJf2fJf(const Eigen::Matrix<double, 6, 6>& bJf, const Eigen::Transform<double, 3, Eigen::Isometry>& bMf,
               Eigen::Matrix<double, 6, 6>& fJf);

  /**
   * 雅可比矩阵求逆
   *
   * @param[in] jac，雅可比矩阵.
   * @param[out] jac_inverse，雅可比逆阵.
   */
  void JacobInverse(const Eigen::Matrix<double, 6, 6>& jac, Eigen::Matrix<double, 6, 6>& jac_inverse);

  /**
   * 计算速度转换矩阵
   *
   * @param[in] fMe，末端相对于法兰的位姿，即TCP位姿.
   * @param[out] fTVe，速度转换矩阵，将相对于TCP的速度转换为相对于法兰的速度.
   */
  void fMe2fTVe(const Eigen::Transform<double, 3, Eigen::Isometry>& fMe, Eigen::Matrix<double, 6, 6>& fTVe);

  /**
   * 根据当前位姿和速度（相对变化量），计算运动单位时间后的位姿
   *
   * @param[in] pose_cur，当前位姿.
   * @param[out] cart_vel，以当前位姿为参考坐标系的笛卡尔空间速度.
   * @param[out] pose_goal，移动过单位时间速度量之后的位姿.
   */
  int CalcCartUpdate(const Eigen::Transform<double, 3, Eigen::Isometry>& pose_cur, const Eigen::Matrix<double, 6, 1>& cart_vel,
                   Eigen::Transform<double, 3, Eigen::Isometry>& pose_goal);
private:
  ApplyVelSixJoints apply_vel_;
  Eigen::Transform<double, 3, Eigen::Isometry> fMe_;   // 末端相对于法兰的位姿，即TCP
  std::array<double, 6>  q_cur_;   //当前关节角
  std::array<double, 16> cart_cur_;  //当前法兰笛卡尔位姿
  Eigen::Transform<double, 3, Eigen::Isometry> bMf_cur_; // 当前法兰位姿
  Eigen::Transform<double, 3, Eigen::Isometry> bMe_cur_; // 当前TCP位姿
  Eigen::Transform<double, 3, Eigen::Isometry> bMe_desire_;  //期望TCP位姿
  Eigen::Matrix<double, 6, 6> fTVe_;  //速度转换矩阵，将相对于TCP的速度转换为相对于法兰的速度
  xmate::JointPositions callback_output_;
  double callback_time_;
  std::shared_ptr<xmate::Robot> robot_;
  std::shared_ptr<xmate::XmateModel> robot_model_;
  std::mutex mutex_;
  Eigen::Matrix<double, 6, 6> bJf_cur_;  //雅可比矩阵，输入关节角速度dq，输出末端相对于基坐标系的速度bVe
  Eigen::Matrix<double, 6, 6> fJf_cur_;  //雅可比矩阵，输入关节角速度dq，输出末端相对于当前末端坐标系的速度fVf.
};

