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

namespace ROS2
{
    class ROS2CameraSensorComponent
        : public ROS2SensorComponent
    {
    public:
        ROS2CameraSensorComponent();
        ~ROS2CameraSensorComponent() = default;
        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", ROS2SensorComponent);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:

        bool SupportsMultipleWindows() {
            return true;
        }

        void InitializeSecondPipeline();
        void OpenSecondSceneWindow();
        void DrawOnSecondWindow();

        void FrequencyTick() override;


        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        AZStd::string m_cameraName = "dummy";
        void ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result);

        void UpdateCamera();

        void DrawAuxGeom() const;

        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::Entity* m_cameraEntity = nullptr;

        AZStd::unique_ptr<AzFramework::EntityContext> m_entityContext;
        AZStd::unique_ptr<class WindowedView> m_windowedView;

        AZ::RPI::Scene* m_scene = nullptr;
    };
}  // namespace ROS2
