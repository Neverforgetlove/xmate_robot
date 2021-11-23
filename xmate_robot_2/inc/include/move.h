/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#include <iostream>
#include <array>

#include "joint_motion.h"
#include "cart_motion.h"
#include "robot.h"

enum class MOVETYPE {
    MOVEJ,
    MOVEL,
    MOVEC  
};

struct cart_pos{
    std::array<double,16> pos;
    bool psi_valid;
    double psi;

    cart_pos(){
        pos.fill(0.0);
        psi_valid = false;
        psi = 0.0;
    }
};

/**
 * @brief MOVEJ指令  
 */
bool MOVEJ(double speed,const std::array<double,7>& q_start,const std::array<double,7>& q_target,xmate::Robot& robot);

/**
 * @brief MOVEL指令  
 */
bool MOVEL(double speed,cart_pos& pos_start, cart_pos& pos_targrt,xmate::Robot& robot);

/**
 * @brief MOVEC指令  
 */
bool MOVEC(double speed,cart_pos& pos_start, cart_pos& pos_mid, cart_pos& pos_target,xmate::Robot& robot);
