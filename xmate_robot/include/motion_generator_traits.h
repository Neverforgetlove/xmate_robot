/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include "control_types.h"
#include "rci_data/command_types.h"

namespace xmate {

template <typename T>
struct MotionGeneratorTraits {};

template <>
struct MotionGeneratorTraits<Torques> {
    static constexpr auto kMotionGeneratorMode = RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle;
};

template <>
struct MotionGeneratorTraits<JointPositions> {
    static constexpr auto kMotionGeneratorMode = RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition;
};

template <>
struct MotionGeneratorTraits<CartesianPose> {
    static constexpr auto kMotionGeneratorMode = RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition;
};

}  // namespace xmate
