/*
 * Copyright 2022 Ladon Robotics
 *
 * This file is was created for the sole use of Ladon Robotics.
 */
/**
 * @brief WingsailActuator plugin
 * @file wingsail_actuator.cpp
 * @author Pierce Nichols <pierce@ladonrobotics.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "sail_interfaces/msg/wingsail_actuator.hpp"

namespace mavros
{
    namespace extra_plugins
    {
        using namespace std::placeholders;      // NOLINT

        /**
         * @brief WingsailActuator plugin
         * @plugin wingsail_actuator
         *
         * Receive wingsail actuator commands
         */
        class WingsailActuatorPlugin : public plugin::Plugin
        {
        public:
            explicit WingsailActuatorPlugin(plugin::UASPtr uas) : Plugin(uas, "wingsail_actuator")
            {
                auto sensor_qos = rclcpp::SensorDataQoS();

                mWingsailActPub = node->create_publisher<sail_interfaces::msg::WingsailActuator>(
                    "wingsail_actuator", sensor_qos);
            }

            Subscriptions get_subscriptions() override
            {
                return {
                    make_handler(&WingsailActuatorPlugin::handle_wingsail)
                };
            }

        private:
            rclcpp::Publisher<sail_interfaces::msg::WingsailActuator>::SharedPtr mWingsailActPub;

            // rx handler
            void handle_wingsail(
                const mavlink::mavlink_message_t * msg [[maybe_unused]],
                mavlink::ladon_robotics::msg::WINGSAIL_ACTUATOR & in,
                plugin::filter::ComponentAndOk filter [[maybe_unused]])
            {
                auto out = sail_interfaces::msg::WingsailActuator();
                out.header.stamp        = node->now();
                out.target_sail         = in.target_sail;
                out.sail_angle_type     = in.sail_angle_type;
                out.sail_angle          = in.sail_angle;
                out.flap_active         = in.flap_active;
                out.flap_angle          = in.flap_angle;

                mWingsailActPub->publish(out);
            }
        };

    } // namespace std_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::WingsailActuatorPlugin)
