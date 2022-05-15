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
#include <AzCore/Math/MatrixUtils.h>

#include <sensor_msgs/image_encodings.hpp>
#include <Utilities/ROS2Names.h>

#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/RenderPipeline.h>

#include <AzFramework/Windowing/WindowBus.h>
#include <AzFramework/Windowing/NativeWindow.h>
#include <Atom/RHI/RHISystemInterface.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/WindowContext.h>

#include <Atom/Component/DebugCamera/CameraComponent.h>
#include <Atom/Component/DebugCamera/NoClipControllerComponent.h>

#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>

#include <PostProcess/PostProcessFeatureProcessor.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
    }

    const char* const tracePrefix = "ROS2CameraSensor";


    ROS2CameraSensorComponent::ROS2CameraSensorComponent() {
        PublisherConfiguration pc;
        auto type = Internal::kImageMessageType;
        pc.m_type = type;
        pc.m_topic = "camera_image";
        m_sensorConfiguration.m_frequency = 10;

        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }


    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                    ->Version(1)
                    ->Field("CameraName", &ROS2CameraSensorComponent::m_cameraName)
                    ->Field("VerticalFieldOfViewDeg", &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg)
                    ->Field("Width", &ROS2CameraSensorComponent::m_width)
                    ->Field("Height", &ROS2CameraSensorComponent::m_height);

            AZ::EditContext* ec = serialize->GetEditContext();
            if (ec) {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Simple Camera component]")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_cameraName, "Camera Name", "This is the camera name.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg, "Vertical field of view", "Camera's vertical (y axis) field of view in degrees.")
                            ->Attribute(AZ::Edit::Attributes::AddNotify, &ROS2CameraSensorComponent::OnCameraParamsChanged)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_width, "Image width", "Image width")
                            ->Attribute(AZ::Edit::Attributes::AddNotify, &ROS2CameraSensorComponent::OnCameraParamsChanged)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_height, "Image height", "Image height")
                            ->Attribute(AZ::Edit::Attributes::AddNotify, &ROS2CameraSensorComponent::OnCameraParamsChanged);
            }
        }
    }
    void ROS2CameraSensorComponent::OnCameraParamsChanged() {
        ApplyParams();
    }

    void ROS2CameraSensorComponent::InitializeSecondPipeline() {
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();

        const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines = m_scene->GetRenderPipelines();
        AZ_TracePrintf(tracePrefix, "Pipelne size: %u", renderingPipelines.size());

        AZStd::string pipelineName = "SecondPipeline";

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";          //Surface shaders render to the "MainCamera" tag
        pipelineDesc.m_name = pipelineName;                 //Sets the debug name for this pipeline
        pipelineDesc.m_rootPassTemplate = "MainPipelineRenderToTexture";    //References a template in AtomSampleViewer\Passes\PassTemplates.azasset
        pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(m_width, m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines2 = m_scene->GetRenderPipelines();
        AZ_TracePrintf(tracePrefix, "Pipelne size: %u", renderingPipelines2.size());

        AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();

        AzFramework::EntityContextId contextId = AzFramework::EntityContextId::CreateNull();
        AzFramework::EntityIdContextQueryBus::EventResult(contextId, GetEntityId(), &AzFramework::EntityIdContextQueries::GetOwningContextId);

        AZ_TracePrintf(tracePrefix, "osm Context id: %s\n", contextId.ToString<AZStd::string>().c_str());

        // rendering pipeline has a tree structure
        m_passHierarchy.push_back(pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        //! To have a fully formed set of view transforms you also need to call SetViewToClipMatrix() to set up the projection.
        m_pipeline->SetDefaultView(m_view);
        m_targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            // This will be set again to mimic the active camera in UpdateView
            fp->SetViewAlias(m_view, m_targetView);
        }
    }

    void ROS2CameraSensorComponent::DestroyPipeline()
    {
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            // Remove view alias introduced in CreatePipeline and UpdateView
            fp->RemoveViewAlias(m_view);
        }
        m_scene->RemoveRenderPipeline(m_pipeline->GetId());
        m_passHierarchy.clear();
        m_pipeline.reset();
        m_view.reset();
        m_targetView.reset();
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        InitializeView();
        ApplyParams();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);

        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), publisherConfig.GetQoS());

        InitializeSecondPipeline();
        m_startTime = std::chrono::steady_clock::now();
    }

    void ROS2CameraSensorComponent::InitializeView() {
        AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
    }

    void ROS2CameraSensorComponent::ApplyParams() {
        m_aspectRatio = static_cast<float>(m_width)/static_cast<float>(m_height);

        AZ_TracePrintf(tracePrefix, "New fov %f", m_VerticalFieldOfViewDeg);
        AZ::Matrix4x4 viewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(viewToClipMatrix, AZ::DegToRad(m_VerticalFieldOfViewDeg), m_aspectRatio, m_nearDist, m_farDist, true);
        m_view->SetViewToClipMatrix(viewToClipMatrix);
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        DestroyPipeline();
        ROS2SensorComponent::Deactivate();
    }

    float ROS2CameraSensorComponent::SecondsSinceStart() {
        return static_cast<float>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - m_startTime).count())/1000.0f;
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();//AZ::Transform::Identity();
        AZ::Vector3 eulerAngles = AZ::ConvertTransformToEulerDegrees(transform);
        AZ::Vector3 translation = transform.GetTranslation();

        AZ_TracePrintf(tracePrefix, "Translation: %f, %f, %f, euler: %f, %f, %f",
                       translation.GetX(), translation.GetY(), translation.GetZ(),
                       eulerAngles.GetX(), eulerAngles.GetY(), eulerAngles.GetZ());

        AZ::Transform inverse = transform.GetInverse();

        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(),
                                                                                       inverse.GetTranslation()));

        AZ::Render::FrameCaptureRequestBus::Broadcast(
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            AZStd::vector<AZStd::string>({"SecondPipeline", "CopyToSwapChain"}),
            AZStd::string("Output"),
            AZStd::bind(&ROS2CameraSensorComponent::ReadbackCallback, this, AZStd::placeholders::_1),
            AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    void ROS2CameraSensorComponent::ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
        AZ_TracePrintf(tracePrefix, "New image %u x %u %s",
                       descriptor.m_size.m_width,
                       descriptor.m_size.m_height,
                       AZ::RHI::ToString(descriptor.m_format));

        sensor_msgs::msg::Image message;
        message.encoding = sensor_msgs::image_encodings::RGBA8;

        message.width = descriptor.m_size.m_width;
        message.height = descriptor.m_size.m_height;
        message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
        message.header.frame_id = "camera";

        m_imagePublisher->publish(message);
    }
} // namespace ROS2
