/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "Sensor/ROS2SensorComponent.h"

#include <AzCore/Component/Component.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>

#include <AzFramework/Entity/EntityContext.h>

#include <chrono>
#include <optional>

namespace ROS2
{
    class ROS2CameraSensorComponent
        : public ROS2SensorComponent
    {
    public:
        ROS2CameraSensorComponent();
        ~ROS2CameraSensorComponent() override = default;
        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", ROS2SensorComponent);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:

        void InitializeSecondPipeline();
        void DestroyPipeline();

        void FrequencyTick() override;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        AZStd::string m_cameraName = "dummy";
        void ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result);

        void UpdateCamera();

        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::ViewPtr m_targetView;
        AZStd::function<void()> m_captureFinishedCallback;

        AZ::RPI::Scene* m_scene = nullptr;

        std::chrono::steady_clock::time_point m_startTime;
        float SecondsSinceStart();
    };
}  // namespace ROS2
