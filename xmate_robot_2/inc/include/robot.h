/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "clients.h"
#include "control_tools.h"
#include "logger.h"
#include "network.h"
#include "rci_data/command_traits.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"

namespace xmate {
class Robot {
    using MoveCallback = std::function<void(char *p)>;

    using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
    using TorqueControl = std::function<Torques(RCI::robot::RobotState robot_state)>;
    using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

   public:
    explicit Robot(const std::string &ip, uint16_t port, bool enforce_realtime = true, size_t log_size = 50);

   public:  // 框架函数
    void startUdp();
    void stopUdp();
    void startTcp();
    void reg();

   public:  // 回调函数，适用于UDP算法交互,用户的控制算法写在这里
    void SetCallback(MoveCallback callback, std::string name);
    void Control(JointControl joint_control);
    void Control(CartesianControl cartesian_control);
    void Control(TorqueControl torque_control);

   protected:
    bool motionGeneratorRunning() const noexcept;
    bool controllerRunning() const noexcept;

    //更新控制模式，不涉及收发
    void updateState(const RCI::robot::RobotState &robot_state);

    /**
     * 对关节角度指令进行限幅滤波.
     * @param[in] 运动控制指令RobotCommand.
     * @param[in] 机器人状态RobotState.
     * @throw Exception If Commanding value is infinite or NaN.
     */

    void convertJointMotion(RCI::robot::RobotCommand &robot_command, const RCI::robot::RobotState &robot_state);

    /**
     * 对笛卡尔空间位姿指令进行限幅滤波.
     * @param[in] 运动控制指令RobotCommand.
     * @param[in] 机器人状态RobotState.
     * @throw Exception If Commanding value is infinite or NaN.
     */

    void convertCartesianMotion(RCI::robot::RobotCommand &robot_command, const RCI::robot::RobotState &robot_state);

    /**
     * 对关节力矩指令进行限幅滤波.
     * @param[in] 运动控制指令RobotCommand.
     * @param[in] 机器人状态RobotState.
     * @throw Exception If Commanding value is infinite or NaN.
     */

    void convertTorqueControl(RCI::robot::RobotCommand &robot_command, const RCI::robot::RobotState &robot_state);

   public:
    /**
     * 若在运动过程中探测到指令控制模式与机器人控制模式不一致，抛出异常数据.
     * @param[in] 机器人状态RobotState.
     * @throw ControlException.
     */
    void throwOnMotionError(const RCI::robot::RobotState &robot_state);

    /**
     * 对RCI指令和机器人状态进行记录.
     * @param[in] 运动控制指令RobotCommand.
     * @param[in] 机器人状态RobotState.
     */
    void log(const RCI::robot::RobotCommand &robot_command, const RCI::robot::RobotState &robot_state);

    /**
     * 写入csv文件
     */
    void writeLogToFile(std::string file_name);

    /**
     * 设置限幅滤波参数.
     * @param[in] 限幅开启时置为true.
     * @param[in] 截止频率cutoff_frequency，范围是10~1000Hz.建议10~100Hz.
     */
    bool setFilterLimit(bool limit_rate, double cutoff_frequency);

    /**
     * 设置机器人上电状态.
     * @param[in] 机器人上电状态，0代表机器人下电，1代表机器人上电
     */
    bool setMotorPower(int motor_state);

    /**
     * 获取机器人上电状态.
     * @param[in] 机器人上电状态，0代表机器人下电，1代表机器人上电
     */
    int getMotorState();

    /**
     * 设置控制和运动模式.
     *
     * 控制模式有:
     * 1关节位置控制模式.
     * 2笛卡尔空间位置控制模式.
     * 3关节阻抗控制模式.
     * 4笛卡尔空间阻抗控制模式.
     * 5力矩控制模式
     *
     * 运动模式有：
     * 1关节空间轴运动
     * 2笛卡尔空间运动
     * 3空(kIdle)
     *
     * 控制模式与运动模式组合起来进行控制
     * 组合方式提供：
     * 1关节空间轴运动+关节位置控制
     * 2关节空间轴运动+关节阻抗控制模式
     * 3笛卡尔空间运动+笛卡尔空间位置控制模式
     * 4笛卡尔空间运动+笛卡尔空间阻抗控制模式
     * 5力矩控制模式
     *
     * @param[in] ControllerMode.
     * @param[in] MotionGeneratorMode.
     */
    void startMove(RCI::robot::StartMoveRequest::ControllerMode controller_mode,
                   RCI::robot::StartMoveRequest::MotionGeneratorMode motion_generator_mode);

    /**
     * 停止运动，停止下发RobotCommand命令.
     */
    void stopMove();

    /**
     * 用于发送RobotCommand指令给机器人.
     * @param[in] 运动控制指令RobotCommand.
     * @param[in] 机器人状态RobotState.
     */
    void sendRobotCommand(RCI::robot::RobotCommand &robot_command, const RCI::robot::RobotState &robot_state);

    /**
     * 接收一次RobotState，不要在运动过程中调用此接口.
     * @return RobotState instance.
     */
    RCI::robot::RobotState receiveRobotState();

    // //不同命名空间的robotstate转换
    // xmate::RobotState convertRobotState(const RCI::robot::RobotState &robot_state) noexcept;

    /**
     * 设置碰撞检测阈值.
     * @param[in] 关节碰撞检测阈值
     */
    void setCollisionBehavior(const std::array<double, 7> &upper_torque_thresholds_nominal);

    /**
     * 设置虚拟墙.
     */
    void setCartesianLimit(const std::array<double, 3> &object_world_size, const std::array<double, 16> &object_frame,
                           const bool &object_activation);

    /**
     * 设置关节阻抗控制系数.
     *
     * @param[in] K_theta 关节阻抗系数 ，max={{1000,1000,1000,1000,500,500,500}}
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setJointImpedance(const std::array<double, 7> &K_theta);

    /**
     * 设置笛卡尔空间阻抗控制系数.
     *
     * @param[in] K_x 笛卡尔空间阻抗控制系数 ,max={{1000,1000,1000,100,100,100}}
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setCartesianImpedance(const std::array<double, 6> &K_x);

    /**
     * 设置末端执行器相对于机器人法兰的位姿.
     *
     * @param[in] F_T_EE 末端执行器坐标系相对于法兰坐标系的转换矩阵
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setCoor(const std::array<double, 16> &F_T_EE);  //

    /**
     * 设置工具和负载的质量、质心和惯性矩阵.
     *
     * @param[in] 工具和负载的质量.
     * @param[in] 工具和负载的质心.
     * @param[in] 工具和负载的惯性矩阵
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setLoad(const double &load_mass, const std::array<double, 3> &load_center,
                 const std::array<double, 9> &load_inertia);

    /**
     * 设置机器人控制器的滤波截止频率.
     * 截止频率范围在10Hz~1000Hz范围内，超过1000Hz视为没有进行滤波.建议设置为10~100Hz.
     *
     * @param[in] 关节位置的滤波截止频率.
     * @param[in] 笛卡尔空间位置的滤波截止频率.
     * @param[in] 关节力矩的滤波截止频率.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     *
     */
    void setFilters(const double &joint_position_filter_frequency, const double &cartesian_position_filter_frequency,
                    const double& torque_filter_frequency);

    /**
     * 笛卡尔空间阻抗时，设置末端期望力.
     * 
     * @param[in] 笛卡尔空间末端期望力.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     *
     */
    void setCartImpDesiredTau(const std::array<double, 6> &tau);

    void setTorqueFilterCutOffFrequency(const double& cutoff_frequency);
    /**
     * 当错误发生后，自动恢复机器人.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void automaticErrorRecovery();

    /**
     * 加载机器人模型参数
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void loadRobotParams(std::array<double,28> &dh_params,std::array<double,21> &rob_dims,
                        std::array<double,66> &dynamic_param_with_friction,
                        std::array<double, 45> &dynamic_param_without_friction);

    void enableVirtualGuide(const bool enable);

    /**
     * 设置机器人IO OUTPUT端信号
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setDO(RCI::robot::DOSIGNAL DO_signal,bool state);

    /**
     * 设置机器人IO INPUT端信号
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    bool getDI(RCI::robot::DISIGNAL DI_signal);

   private:
    template <typename R, typename T>
    R tcpCmdSend(T &data) {
        return network->tcpCmdBlockHandshake<R, T>(data);
    }

    template <typename T>
    void handleCommandResponse(const T &response) const;

   private:
    std::shared_ptr<Network> network;
    NetThread *p_udp;
    NetThread *p_tcp;

    RCI::robot::MotionGeneratorMode motion_generator_mode_;
    RCI::robot::MotionGeneratorMode current_move_motion_generator_mode_ = RCI::robot::MotionGeneratorMode::kIdle;
    RCI::robot::ControllerMode controller_mode_ = RCI::robot::ControllerMode::kOther;
    RCI::robot::ControllerMode current_move_controller_mode_;

    //滤波限幅度参数，
    bool limit_rate_;
    double cutoff_frequency_;
    Logger logger_;
    RCI::robot::RobotCommand robot_command_last_;
    uint64_t m_mess_id;

    JointControl m_joint_control;
    CartesianControl m_cartesian_control;
    TorqueControl m_torque_control;


   public:
    uint64_t initial_time_;
    std::array<double, 7> initial_position_;
    
};

template <>
void Robot::handleCommandResponse<RCI::robot::StartMoveResponse>(const RCI::robot::StartMoveResponse &response) const;

template <>
void Robot::handleCommandResponse<RCI::robot::StopMoveResponse>(const RCI::robot::StopMoveResponse &response) const;

template <>
void Robot::handleCommandResponse<RCI::robot::LoadRobotModelResponse>(
    const RCI::robot::LoadRobotModelResponse &response) const;

template <>
void Robot::handleCommandResponse<RCI::robot::EnableVirtualGuideResponse>(
    const RCI::robot::EnableVirtualGuideResponse &response) const;

template <>
void Robot::handleCommandResponse<RCI::robot::AutomaticErrorRecoveryResponse>(
    const RCI::robot::AutomaticErrorRecoveryResponse &response) const;

template <typename T, size_t N>
inline void checkFinite(const std::array<T, N> &array) {
    if (!std::all_of(array.begin(), array.end(), [](double d) { return std::isfinite(d); })) {
        throw std::invalid_argument("Commanding value is infinite or NaN.");
    }
}

inline void checkMatrix(const std::array<double, 16> &transform) {
    checkFinite(transform);
    if (!isHomogeneousTransformation(transform)) {
        throw std::invalid_argument(
            "libxmate: Attempt to set invalid transformation in motion generator. Has to be column "
            "major!");
    }
}

}  // namespace xmate
