/*
 * tracer_base.hpp
 *
 * Created on: Apr 14, 2020 10:21
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_HPP
#define TRACER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/details/interface/tracer_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
class TracerBaseV2 : public AgilexBase<ProtocolV2Parser>,
                     public TracerInterface {
 public:
  TracerBaseV2() : AgilexBase<ProtocolV2Parser>(){};
  ~TracerBaseV2() = default;

  // set up connection
  bool Connect(std::string can_name) override {
    return AgilexBase<ProtocolV2Parser>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ProtocolV2Parser>::SendMotionCommand(linear_vel, angular_vel,
                                                    0.0, 0.0);
  }

  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value) override {
    AgilexBase<ProtocolV2Parser>::SendLightCommand(f_mode, f_value, CONST_OFF,
                                                   0);
  }

  void DisableLightControl() override {
    AgilexBase<ProtocolV2Parser>::DisableLightControl();
  }

  // get robot state
  TracerCoreState GetRobotState() override {
    auto state = AgilexBase<ProtocolV2Parser>::GetRobotCoreStateMsgGroup();

    TracerCoreState tracer_state;
    tracer_state.time_stamp = state.time_stamp;
    tracer_state.system_state = state.system_state;
    tracer_state.motion_state = state.motion_state;
    tracer_state.light_state = state.light_state;
    tracer_state.rc_state = state.rc_state;
    return tracer_state;
  }

  TracerActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ProtocolV2Parser>::GetActuatorStateMsgGroup();

    TracerActuatorState tracer_actuator;
    tracer_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 2; ++i) {
      tracer_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      tracer_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    }
    return tracer_actuator;
  }

  TracerCommonSensorState GetCommonSensorState() override {
    auto common_sensor =
        AgilexBase<ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    TracerCommonSensorState tracer_bms;

    tracer_bms.time_stamp = common_sensor.time_stamp;

    tracer_bms.bms_basic_state.current = common_sensor.bms_basic_state.current;
    // Note: BMS CAN message definition is not consistent across AgileX robots.
    // Robots with steering mechanism should additionally divide the voltage by
    // 10.
    tracer_bms.bms_basic_state.voltage =
        common_sensor.bms_basic_state.voltage * 0.1f;
    tracer_bms.bms_basic_state.battery_soc =
        common_sensor.bms_basic_state.battery_soc;
    tracer_bms.bms_basic_state.battery_soh =
        common_sensor.bms_basic_state.battery_soh;
    tracer_bms.bms_basic_state.temperature =
        common_sensor.bms_basic_state.temperature;
    
    tracer_bms.charge_state.charge_flag = 
        common_sensor.charge_state.charge_flag;

    return tracer_bms;
  }

  void ResetRobotState() override {
    AgilexBase<ProtocolV2Parser>::ResetRobotState();
  }
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
