/*
 * Copyright 2023 Ladon Robotics
 *
 * This file is was created for the sole use of Ladon Robotics.
 */
/**
 * @brief WingsailFeedback plugin
 * @file wingsail_feedback.cpp
 * @author Pierce Nichols <pierce@ladonrobotics.com>
 *
 * @addtogroup plugin
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "angles/angles.h"

#include "ssp_interfaces/msg/wingsail_feedback.hpp"

namespace mavros
{
    namespace extra_plugins
    {
        using namespace std::placeholders;      // NOLINT

        /**
         * @brief WingsailFeedback plugin
         * @plugin wingsail_feedback
         *
         * Transmit wingsail feedback messages
         */
        class WingsailFeedbackPlugin : public plugin::Plugin
        {
        public:
            explicit WingsailFeedbackPlugin(plugin::UASPtr uas) : Plugin(uas, "wingsail_feedback")
            {
                auto sensor_qos = rclcpp::SensorDataQoS();

                mSub = node->create_subscription<ssp_interfaces::msg::WingsailFeedback>(
                    "feedback", sensor_qos, std::bind(&WingsailFeedbackPlugin::cb, this, _1));
            }

            Subscriptions get_subscriptions() override
            {
                return {
                    make_handler(&WingsailFeedbackPlugin::handle_wingsail)
                };
            }

        private:
            rclcpp::Subscription<ssp_interfaces::msg::WingsailFeedback>::SharedPtr mSub;

            // rx handler (null)
            void handle_wingsail(
                const mavlink::mavlink_message_t * msg [[maybe_unused]],
                mavlink::ladon_robotics::msg::WINGSAIL_FEEDBACK & in [[maybe_unused]],
                plugin::filter::ComponentAndOk filter [[maybe_unused]])
            {}

            // callback
            void cb (const ssp_interfaces::msg::WingsailFeedback::SharedPtr in)
            {
                mavlink::ladon_robotics::msg::WINGSAIL_FEEDBACK out{};
                out.source_sail     = in->source_sail;
                out.sail_angle      = in->sail_angle;
                out.wind_angle      = in->wind_angle;
                out.flap_active     = in->flap_active;
                for (int i = 0; i < 8; i++)
                {
                    out.flap_angle[i] = (in->flap_active & (1 << i)) ? in->flap_angle[i] : NAN;
                }

                uas->send_message(out);
            }
        };

    } // namespace std_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::WingsailFeedbackPlugin)
