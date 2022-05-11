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

#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/RenderPipeline.h>

#include <AzFramework/Windowing/WindowBus.h>
#include <AzFramework/Windowing/NativeWindow.h>
#include <Atom/RHI/RHISystemInterface.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/WindowContext.h>

#include "EntityUtilityFunctions.h"

#include <Atom/Component/DebugCamera/CameraComponent.h>
#include <Atom/Component/DebugCamera/NoClipControllerComponent.h>

#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>


namespace ROS2
{

    //! A simple example for how to set up a second window, view, renderPipeline, and basic entities.
    class WindowedView final
            : public AzFramework::WindowNotificationBus::Handler
    {
        friend ROS2CameraSensorComponent;
    protected:
        AZStd::unique_ptr<AzFramework::NativeWindow> m_nativeWindow;
        AZStd::shared_ptr<AZ::RPI::WindowContext> m_windowContext;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::Entity* m_cameraEntity = nullptr;
        AZ::RPI::ViewPtr m_view;
        ROS2CameraSensorComponent* m_parent;

    public:
        WindowedView(AzFramework::EntityContextId contextId, ROS2CameraSensorComponent *parent)
                : m_parent(parent)
        {
            m_parent = parent;

            // Create a NativeWindow and WindowContext
            m_nativeWindow = AZStd::make_unique<AzFramework::NativeWindow>("Multi View Single Scene: Second Window", AzFramework::WindowGeometry(0, 0, 640, 480));
            m_nativeWindow->Activate();
            AZ::RHI::Ptr<AZ::RHI::Device> device = AZ::RHI::RHISystemInterface::Get()->GetDevice();
            m_windowContext = AZStd::make_shared<AZ::RPI::WindowContext>();
            m_windowContext->Initialize(*device, m_nativeWindow->GetWindowHandle());

            AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();

            AZ::RPI::Scene* scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));
            const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines = scene->GetRenderPipelines();
            AZ_TracePrintf(">>>>>>>>>>>", "Pipelne size: %u", renderingPipelines.size());
//            for (const auto& pipeline : renderingPipelines) {
//                AZ_TracePrintf(">>>>>>>>>>>", "Pipelne size: %u", pipeline->);
//            }

            // Create a custom pipeline descriptor
            AZ::RPI::RenderPipelineDescriptor pipelineDesc;
            pipelineDesc.m_mainViewTagName = "MainCamera";          //Surface shaders render to the "MainCamera" tag
            pipelineDesc.m_name = "SecondPipeline";                 //Sets the debug name for this pipeline
            pipelineDesc.m_rootPassTemplate = "MainPipeline";    //References a template in AtomSampleViewer\Passes\PassTemplates.azasset
            pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
            m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipelineForWindow(pipelineDesc, *m_windowContext);

            AZ_TracePrintf(">>>>>>>>>>>", "New pipelne %p", m_pipeline.get());

            scene->AddRenderPipeline(m_pipeline);

            const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines2 = scene->GetRenderPipelines();
            AZ_TracePrintf(">>>>>>>>>>>", "Pipelne size: %u", renderingPipelines2.size());

            AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();

            // Create a camera entity, hook it up to the RenderPipeline
            m_cameraEntity = AtomSampleViewer::CreateEntity("WindowedViewCamera", contextId);
            AZ::Debug::CameraComponentConfig cameraConfig(m_windowContext);
            cameraConfig.m_fovY = AZ::Constants::QuarterPi;
            AZ::Debug::CameraComponent* camComponent = static_cast<AZ::Debug::CameraComponent*>(m_cameraEntity->CreateComponent(azrtti_typeid<AZ::Debug::CameraComponent>()));
            camComponent->SetConfiguration(cameraConfig);
            m_cameraEntity->CreateComponent(azrtti_typeid<AzFramework::TransformComponent>());
            m_cameraEntity->CreateComponent(azrtti_typeid<AZ::Debug::NoClipControllerComponent>());
            m_cameraEntity->Activate();
            m_pipeline->SetDefaultViewFromEntity(m_cameraEntity->GetId());
            m_view = camComponent->GetView();

            AzFramework::WindowNotificationBus::Handler::BusConnect(m_nativeWindow->GetWindowHandle());
        }

        ~WindowedView()
        {
            AzFramework::WindowNotificationBus::Handler::BusDisconnect(m_nativeWindow->GetWindowHandle());

            AtomSampleViewer::DestroyEntity(m_cameraEntity);

            m_pipeline->RemoveFromScene();
            m_pipeline = nullptr;

            m_windowContext->Shutdown();
            m_windowContext = nullptr;
        }

        // AzFramework::WindowNotificationBus::Handler overrides ...
        void OnWindowClosed() override
        {
            // TODO: !!! USE THAT ASAP !!!
//            m_parent->OnChildWindowClosed();
        }

        AzFramework::NativeWindowHandle GetNativeWindowHandle()
        {
            if (m_nativeWindow)
            {
                return m_nativeWindow->GetWindowHandle();
            }
            else
            {
                return nullptr;
            }
        }
    };

    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
    }

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

    void ROS2CameraSensorComponent::InitializeSecondPipeline() {
        AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();

        const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines = m_scene->GetRenderPipelines();
        AZ_TracePrintf(">>>>>>>>>>>", "Pipelne size: %u", renderingPipelines.size());


        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";          //Surface shaders render to the "MainCamera" tag
        pipelineDesc.m_name = "SecondPipeline";                 //Sets the debug name for this pipeline
        pipelineDesc.m_rootPassTemplate = "MainPipelineRenderToTexture";    //References a template in AtomSampleViewer\Passes\PassTemplates.azasset
        pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);


        AZ_TracePrintf(">>>>>>>>>>>", "New pipelne %p", m_pipeline.get());

        m_scene->AddRenderPipeline(m_pipeline);

        const AZStd::vector<AZStd::shared_ptr<AZ::RPI::RenderPipeline>>& renderingPipelines2 = m_scene->GetRenderPipelines();
        AZ_TracePrintf(">>>>>>>>>>>", "Pipelne size: %u", renderingPipelines2.size());

        AZ::RPI::PassSystemInterface::Get()->DebugPrintPassHierarchy();


        AzFramework::EntityContextId contextId = AzFramework::EntityContextId::CreateNull();
        AzFramework::EntityIdContextQueryBus::EventResult(contextId, GetEntityId(), &AzFramework::EntityIdContextQueries::GetOwningContextId);

        AZ_TracePrintf(">>>>>>>>>>>", "osm Context id: %s\n", contextId.ToString<AZStd::string>().c_str());


//        m_cameraEntity = AtomSampleViewer::CreateEntity("WindowedViewCamera", contextId);
//        AZ::Debug::CameraComponentConfig cameraConfig(m_windowContext);
//        cameraConfig.m_fovY = AZ::Constants::QuarterPi;
//        AZ::Debug::CameraComponent* camComponent = static_cast<AZ::Debug::CameraComponent*>(m_cameraEntity->CreateComponent(azrtti_typeid<AZ::Debug::CameraComponent>()));
//        camComponent->SetConfiguration(cameraConfig);
//        m_cameraEntity->CreateComponent(azrtti_typeid<AzFramework::TransformComponent>());
//        m_cameraEntity->CreateComponent(azrtti_typeid<AZ::Debug::NoClipControllerComponent>());
//        m_cameraEntity->Activate();
//        m_pipeline->SetDefaultViewFromEntity(m_cameraEntity->GetId());
//        m_view = camComponent->GetView();
//
//        AzFramework::WindowNotificationBus::Handler::BusConnect(m_nativeWindow->GetWindowHandle());
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);

        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), publisherConfig.GetQoS());

        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));
//        for (const auto& pipeline : renderingPipelines) {
//            AZ_TracePrintf(">>>>>>>>>>>", "Pipeline name: %s", pipeline->GetRootPass())
//        }
        OpenSecondSceneWindow();
//        InitializeSecondPipeline();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        DrawOnSecondWindow();
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
        AZ_TracePrintf(">>>>>>>>>>>", "New image %u x %u", descriptor.m_size.m_width, descriptor.m_size.m_height);

        sensor_msgs::msg::Image message;
        message.encoding = sensor_msgs::image_encodings::BGRA8;

        message.width = descriptor.m_size.m_width;
        message.height = descriptor.m_size.m_height;
        message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
        message.header.frame_id = "camera";

        m_imagePublisher->publish(message);
    }

    void ROS2CameraSensorComponent::OpenSecondSceneWindow()
    {
        if (SupportsMultipleWindows() && !m_windowedView)
        {
            AzFramework::EntityContextId contextId = AzFramework::EntityContextId::CreateNull();
            AzFramework::EntityIdContextQueryBus::EventResult(contextId, GetEntityId(), &AzFramework::EntityIdContextQueries::GetOwningContextId);
            AZ_TracePrintf(">>>>>>>>>>>", "osm Context id: %s\n", contextId.ToString<AZStd::string>().c_str());

            m_windowedView = AZStd::make_unique<WindowedView>(contextId, this);
        }
    }

    void ROS2CameraSensorComponent::DrawOnSecondWindow() {
        DrawAuxGeom();

//        if (SupportsMultipleWindows() && ImGui::Begin("Multi View Panel"))
//        {
//            if(m_windowedView)
//            {
//                if (ImGui::Button("Close Second View Window"))
//                {
//                    m_windowedView = nullptr;
//                }
//            }
//            else
//            {
//                if (ImGui::Button("Open Second View Window"))
//                {
//                    OpenSecondSceneWindow();
//                }
//            }
//            ImGui::End();
//        }

        if (m_windowedView)
        {
//            float fovRadians = AZ::Constants::QuarterPi;
//            float nearClipDistance = 0.1f;
//            float farClipDistance = 100.0f;

            AZ::EntityId secondCameraEntityId = m_windowedView->m_cameraEntity->GetId();
            AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();//AZ::Transform::Identity();
            AZ::TransformBus::Event(secondCameraEntityId, &AZ::TransformBus::Events::SetWorldTM, transform);
//            Camera::CameraComponentRequests* secondCamInterface = Camera::CameraRequestBus::FindFirstHandler(secondCameraEntityId);
//            secondCamInterface->SetFovRadians(fovRadians);
//            secondCamInterface->SetNearClipDistance(nearClipDistance);
//            secondCamInterface->SetFarClipDistance(farClipDistance);
        }
    }

    void ROS2CameraSensorComponent::DrawAuxGeom() const
    {
//        auto auxGeomFP = m_scene->GetFeatureProcessor<AZ::RPI::AuxGeomFeatureProcessorInterface>();
//        if (auto auxGeom = auxGeomFP->GetDrawQueue())
//        {
//            DrawBackgroundBox(auxGeom);
//
//            DrawThreeGridsOfPoints(auxGeom);
//
//            DrawAxisLines(auxGeom);
//
//            DrawLines(auxGeom);
//
//            DrawBoxes(auxGeom, -20.0f);
//
//            Draw2DWireRect(auxGeom, AZ::Colors::Red, 1.0f);
//        }
//
//        if (m_windowedView)
//        {
//            if (auto auxGeom = auxGeomFP->GetDrawQueueForView(m_windowedView->m_view.get()))
//            {
//                DrawTriangles(auxGeom);
//
//                DrawShapes(auxGeom);
//
//                DrawBoxes(auxGeom, 10.0f);
//
//                DrawDepthTestPrimitives(auxGeom);
//
//                Draw2DWireRect(auxGeom, AZ::Colors::Yellow, 0.9f);
//            }
//        }
    }

} // namespace ROS2
