/*
 * Copyright 2022 Ladon Robotics
 *
 * This file is was created for the sole use of Ladon Robotics.
 */
/**
 * @brief WindData plugin
 * @file wind_data.cpp
 * @author Pierce Nichols <pierce@ladonrobotics.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "angles/angles.h"

#include "ssp_interfaces/msg/wind_data.hpp"
#include "ssp_interfaces/ssp_interfaces.hpp"

using namespace angles;

namespace mavros
{
    namespace extra_plugins
    {
        using namespace std::placeholders;      // NOLINT

        /**
         * @brief WindData plugin
         * @plugin wind_data
         *
         * Receive and transmit wind data
         */
        class WindDataPlugin : public plugin::Plugin
        {
        public:
            explicit WindDataPlugin(plugin::UASPtr uas) : Plugin(uas, "wingsail_actuator")
            {
                auto sensor_qos = rclcpp::SensorDataQoS();

                mWindDataPub = node->create_publisher<ssp_interfaces::msg::WindData>(
                    "wind_data_in", sensor_qos);
                mWindDataSub = node->create_subscription<ssp_interfaces::msg::WindData>(
                    "wind_data_out", sensor_qos, std::bind(
                        &WindDataPlugin::wind_cb, this, _1));
            }

            Subscriptions get_subscriptions() override
            {
                return {
                    make_handler(&WindDataPlugin::handle_wind)
                };
            }

        private:
            rclcpp::Publisher<ssp_interfaces::msg::WindData>::SharedPtr mWindDataPub;
            rclcpp::Subscription<ssp_interfaces::msg::WindData>::SharedPtr mWindDataSub;

            // rx handler
            void handle_wind(
                const mavlink::mavlink_message_t * msg [[maybe_unused]],
                mavlink::ladon_robotics::msg::WIND_DATA & in,
                plugin::filter::ComponentAndOk filter [[maybe_unused]])
            {
                auto out = ssp_interfaces::msg::WindData();
                out.header.stamp    = node->now();
                out.source_sail     = in.source_sail;
                out.wind_type       = in.wind_type;
                out.speed           = in.speed;
                out.direction       = in.direction;

                mWindDataPub->publish(out);
            }

            // callback
            void wind_cb(const ssp_interfaces::msg::WindData::SharedPtr in)
            {
                mavlink::ladon_robotics::msg::WIND_DATA out{};
                mavlink::common::msg::WIND_COV out_cv{};
                out.source_sail     = in->source_sail;
                out.wind_type       = in->wind_type;
                out.speed           = in->speed;
                out.direction       = in->direction;
                out_cv.wind_x       = in->speed * cos(from_degrees(in->direction));
                out_cv.wind_y       = in->speed * sin(from_degrees(in->direction));

                uas->send_message(out);
                if (in->wind_type == (uint8_t)ssp_interfaces::WIND_DATA_TYPE::WIND_DATA_TYPE_TRUE) { uas->send_message(out_cv); }
            }
        };

    } // namespace std_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::WindDataPlugin)
