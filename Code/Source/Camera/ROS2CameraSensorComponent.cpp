/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Camera/ROS2CameraSensorComponent.h"
#include "ROS2/ROS2Bus.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <sensor_msgs/image_encodings.hpp>

namespace ROS2
{
    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                    ->Version(1)
                    ->Field("CameraName", &ROS2CameraSensorComponent::m_cameraName);

            AZ::EditContext* ec = serialize->GetEditContext();
            if (ec) {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Simple Camera component]")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_cameraName, "Camera Name", "This is the camera name.");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        auto config = GetConfiguration();

        // TODO - also use QoS
        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(GetFullTopic().data(), 10);
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        sensor_msgs::msg::Image message;
        message.encoding = sensor_msgs::image_encodings::RGB8;
        const size_t numChannels = sensor_msgs::image_encodings::numChannels(message.encoding);

        message.width = 10;
        message.height = 10;

        size_t dataSize = message.width * message.height * numChannels;
        message.data = std::vector<uint8_t>(dataSize, 0);

        for (int i = 0; i < message.width * message.height; i++) {
            size_t idx = i * numChannels;
            message.data[idx] = 0;
            message.data[idx + 1] = i <= 50 ? 255 : 0;
            message.data[idx + 2] = i >= 50 ? 255 : 0;
        }
        message.header.frame_id = "camera";

        m_imagePublisher->publish(message);
    }
} // namespace ROS2
