/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Camera/ROS2CameraSensorComponent.h"
#include "ROS2/ROS2Bus.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

#include <sensor_msgs/image_encodings.hpp>
#include <Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
    }

    ROS2CameraSensorComponent::ROS2CameraSensorComponent() {
        auto pc = AZStd::make_shared<PublisherConfiguration>();
        auto type = Internal::kImageMessageType;
        pc->m_type = type;
        pc->m_topic = "camera_image";
        m_sensorConfiguration.m_frequency = 10;

        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }
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
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig->m_topic);

        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), publisherConfig->GetQoS());
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Render::FrameCaptureRequestBus::Broadcast(
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            AZStd::vector<AZStd::string>(1, "CopyToSwapChain"),
            AZStd::string("Output"),
            AZStd::bind(&ROS2CameraSensorComponent::ReadbackCallback, this, AZStd::placeholders::_1),
            AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    void ROS2CameraSensorComponent::ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;

        sensor_msgs::msg::Image message;
        message.encoding = sensor_msgs::image_encodings::BGRA8;

        message.width = descriptor.m_size.m_width;
        message.height = descriptor.m_size.m_height;
        message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
        message.header.frame_id = "camera";

        m_imagePublisher->publish(message);
    }
} // namespace ROS2
