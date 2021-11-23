/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <array>
#include <vector>

#include "robot.h"

namespace Model {
class RobotModel;
}

namespace xmate {

struct Load {
    double mass;
    std::array<double, 3> load_center;
    std::array<double, 9> load_interia;
};

enum class XmateType{
    XMATE3,
    XMATE7
};

/**
 * 坐标系枚举
 */
enum class SegmentFrame {
    kJoint1,
    kJoint2,
    kJoint3,
    kJoint4,
    kJoint5,
    kJoint6,
    kJoint7,
    kFlange,
    kEndEffector,
    kStiffness
};

SegmentFrame operator++(SegmentFrame &frame, int /* dummy */) noexcept;

enum class TauType { TAU_FULL, TAU_INERTIAL, TAU_CORIOLIS, TAU_FRICTION, TAU_GRAVITY };

class XmateModel {
   public:
    XmateModel(xmate::Robot *robot_ptr,XmateType model_type);
    ~XmateModel() {}

    Model::RobotModel *GetXmate() { return m_xmate_model; };
    void LoadRobotParams(Robot *robot_ptr);
    void SetLord(double &mass, std::array<double, 3> &cog, std::array<double, 9> &inertia);
    void SetCoor(std::array<double, 16> &F_T_EE, std::array<double, 16> &EE_T_K);

    /**
     * 获取笛卡尔空间位置
     *
     * @param[in] 需要计算笛卡尔位姿的关节角度
     * @param[in] 指定坐标系
     *
     * @return Vectorized 4x4 pose matrix, 行优先.
     */
    std::array<double, 16> GetCartPose(std::array<double, 7> &q, SegmentFrame nr = SegmentFrame::kJoint7);

    /**
     * 获取笛卡尔空间位置
     *
     * @param[in] 需要计算笛卡尔位姿的关节角度
     * @param[in] 末端执行器相对于法兰的位姿
     * @param[in] 刚度坐标系相对于末端执行器的位姿
     * @param[in] 指定坐标系
     *
     * @return Vectorized 4x4 pose matrix, 行优先.
     */
    std::array<double, 16> GetCartPose(

        std::array<double, 7> &q, const std::array<double, 16> &F_T_EE, const std::array<double, 16> &EE_T_K,
        SegmentFrame nr = SegmentFrame::kJoint7);

    /**
     * 获取笛卡尔空间速度
     *
     * @param[in] 需要计算笛卡尔空间速度的关节角度
     * @param[in] 需要计算笛卡尔空间速度的关节角速度
     * @param[in] 指定坐标系
     *
     * @return Array 6x1 twist.
     */

    std::array<double, 6> GetCartVel(std::array<double, 7> &q, std::array<double, 7> &dq,
                                     SegmentFrame nr = SegmentFrame::kJoint7);

    /**
     * 获取笛卡尔空间加速度
     *
     * @param[in] 需要计算笛卡尔空间速度的关节角度
     * @param[in] 需要计算笛卡尔空间速度的关节角速度
     * @param[in] 需要计算笛卡尔空间速度的关节角加速度
     * @param[in] 指定坐标系
     *
     * @return Array 6x1 twist.
     */

    std::array<double, 6> GetCartAcc(std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                                     SegmentFrame nr = SegmentFrame::kJoint7);

    /**
     * 逆解获得关节空间位置
     *
     * @param[in] 法兰笛卡尔空间位姿
     * @param[in] 初始关节角度
     *
     * @return Array 7x1 position.
     */

    std::array<double, 7> GetJointPos(std::array<double, 16> &CartPos, double psi, std::array<double, 7> &q_init);

    /**
     * 逆解获得关节空间速度
     *
     * @param[in] 法兰笛卡尔空间速度
     * @param[in] 此时关节角度
     *
     * @return Array 7x1 velocity.
     */

    std::array<double, 7> GetJointVel(std::array<double, 6> &CartVel, std::array<double, 7> &q);

    /**
     * 逆解获得关节空间加速度
     *
     * @param[in] 法兰笛卡尔空间加速度
     * @param[in] 此时关节角度
     * @param[in] 此时关节角速度
     * @return Array 7x1 acceleration.
     */

    std::array<double, 7> GetJointAcc(std::array<double, 6> &CartAcc, std::array<double, 7> &q,
                                      std::array<double, 7> &dq);

    /**
     * 获取6x7指定坐标系相对于基坐标系的雅克比矩阵 行优先
     *
     * @param[in] 关节角度.
     * @param[in] 指定坐标系
     *
     * @return Array 42x1 Jacobian.
     */
    std::array<double, 42> Jacobian(std::array<double, 7> &q, SegmentFrame nr = SegmentFrame::kJoint7);
    ;

    /**
     * 获取6x7指定坐标系相对于基坐标系的雅克比矩阵 行优先
     *
     * @param[in] 关节角度.
     * @param[in] 末端执行器相对于法兰坐标系的位姿.
     * @param[in] 刚度坐标系相对于末端执行器的位姿.
     * @param[in] 指定坐标系
     *
     * @return Array 42x1 Jacobian.
     */

    std::array<double, 42> Jacobian(std::array<double, 7> &q, const std::array<double, 16> &F_T_EE,
                                    const std::array<double, 16> &EE_T_K, SegmentFrame nr = SegmentFrame::kJoint7);

    /**
     * 由模型计算关节力矩 [Nm]
     *
     * @param[in] 关节角度.
     * @param[in] 关节角速度.
     * @param[in] 关节角加速度.
     * @param[in] 指定力矩类型.
     *
     * @return Array 7x1 Tau.
     */

    std::array<double, 7> GetTau(std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                                 TauType T);

    /**
     * 由模型计算关节力矩 [Nm]
     *
     * @param[in] 末端负载质量.
     * @param[in] 末端负载质心.
     * @param[in] 末端负载惯量.
     * @param[in] 关节角度.
     * @param[in] 关节角速度.
     * @param[in] 关节角加速度.
     * @param[in] 总关节力矩.
     * @param[in] 离心力.
     * @param[in] 科氏力.
     * @param[in] 关节摩擦力.
     * @param[in] 重力矩.
     *
     */
    void GetTauWithFriction(double &mass, std::array<double, 3> &cog, std::array<double, 9> &inertia,
                            std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                            std::array<double, 7> &trq_full, std::array<double, 7> &trq_inertial,
                            std::array<double, 7> &trq_coriolis, std::array<double, 7> &trq_friction,
                            std::array<double, 7> &trq_gravity);

    /**
     * 由模型计算关节力矩 [Nm]
     *
     * @param[in] 关节角度.
     * @param[in] 关节角速度.
     * @param[in] 关节角加速度.
     * @param[in] 总关节力矩.
     * @param[in] 离心力.
     * @param[in] 科氏力.
     * @param[in] 关节摩擦力.
     * @param[in] 重力矩.
     *
     */
    void GetTauWithFriction(std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                            std::array<double, 7> &trq_full, std::array<double, 7> &trq_inertial,
                            std::array<double, 7> &trq_coriolis, std::array<double, 7> &trq_friction,
                            std::array<double, 7> &trq_gravity);

    /**
     * 由模型计算无摩擦力的关节力矩 [Nm]
     *
     * @param[in] 末端负载质量.
     * @param[in] 末端负载质心.
     * @param[in] 末端负载惯量.
     * @param[in] 关节角度.
     * @param[in] 关节角速度.
     * @param[in] 关节角加速度.
     * @param[in] 总关节力矩.
     * @param[in] 离心力.
     * @param[in] 科氏力.
     * @param[in] 关节摩擦力.
     * @param[in] 重力矩.
     *
     */
    void GetTauNoFriction(double &mass, std::array<double, 3> &cog, std::array<double, 9> &inertia,
                          std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                          std::array<double, 7> &trq_full, std::array<double, 7> &trq_inertial,
                          std::array<double, 7> &trq_coriolis, std::array<double, 7> &trq_gravity);

    /**
     * 由模型计算无摩擦力的关节力矩 [Nm]
     *
     * @param[in] 关节角度.
     * @param[in] 关节角速度.
     * @param[in] 关节角加速度.
     * @param[in] 总关节力矩.
     * @param[in] 离心力.
     * @param[in] 科氏力.
     * @param[in] 关节摩擦力.
     * @param[in] 重力矩.
     *
     */

    void GetTauNoFriction(std::array<double, 7> &q, std::array<double, 7> &dq, std::array<double, 7> &ddq,
                          std::array<double, 7> &trq_full, std::array<double, 7> &trq_inertial,
                          std::array<double, 7> &trq_coriolis, std::array<double, 7> &trq_gravity);

   private:
    // Model::RobotModel *m_xmate_model;
    Model::RobotModel *m_xmate_model;
    xmate::Robot *m_robot_ptr;
    Load m_load_param;
    std::array<double, 16> m_F_T_EE;
    std::array<double, 16> m_EE_T_K;
    std::array<double, 21> m_friction_coeff;
    std::vector<double> m_dyn_param_wf;
    std::vector<double> m_dyn_param_nf;
    std::vector<double> m_dh_param;
    std::vector<double> m_rob_dim;
};

}  // namespace xmate
