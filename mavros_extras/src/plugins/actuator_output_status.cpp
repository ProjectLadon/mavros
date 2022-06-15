/*
 * Copyright 2022 Pierce Nichols <pierce@ladonrobotics.com>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 * 
 * This file is a modification of the previous ActuatorControl plugin written
 * by Marcel Stüttgen <stuettgen@fh-aachen.de>
 */
/**
 * @brief ActuatorOutputStatus plugin
 * @file actuator_output_status.cpp
 * @author Pierce Nichols <pierce@ladonrobotics.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/actuator_output_status.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief ActuatorOutputStatus plugin
 * @plugin actuator_output_status
 *
 * Get actuator outputs from the FCU controller.
 */
class ActuatorOutputStatusPlugin : public plugin::Plugin
{
public:
  explicit ActuatorOutputStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "actuator_output_status")
  {
    auto sensor_qos = rclcpp::SensorDataQoS();

    actuator_output_status_pub = node->create_publisher<mavros_msgs::msg::ActuatorOutputStatus>(
      "actuator_output_status", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&ActuatorOutputStatusPlugin::handle_actuator_output_status_target),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::ActuatorOutputStatus>::SharedPtr actuator_output_status_pub;

  /* -*- rx handlers -*- */

  void handle_actuator_output_status_target(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::ACTUATOR_OUTPUT_STATUS & act,
    plugin::filter::ComponentAndOk filter [[maybe_unused]])
  {
    auto ract = mavros_msgs::msg::ActuatorOutputStatus();
    ract.header.stamp = uas->synchronise_stamp(act.time_usec);
    ract.active = act.active;
    for (int i = 0; i < 32; i++) { ract.actuator[i] = act.actuator[i]; }

    actuator_output_status_pub->publish(ract);
  }

};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::ActuatorOutputStatusPlugin)
