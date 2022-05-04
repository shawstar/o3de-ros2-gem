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

#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Pass/FullscreenTrianglePass.h>
#include <Atom/RPI.Public/Image/ImageSystemInterface.h>
#include <Atom/RPI.Public/Image/AttachmentImagePool.h>
#include <Atom/RPI.Public/RPIUtils.h>

#include <Atom/Feature/Utils/FrameCaptureBus.h>

#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/RPI.Reflect/Pass/FullscreenTrianglePassData.h>

#include <sensor_msgs/image_encodings.hpp>

#include <iostream>

namespace ROS2
{

    static const char* s_readbackPipelineTemplate = "ReadbackPipelineTemplate";
    static const char* s_fillerPassTemplate = "ReadbackFillerPassTemplate";
    static const char* s_previewPassTemplate = "ReadbackPreviewPassTemplate";

    static const char* s_fillerShaderPath = "Shaders/Readback/Filler.azshader";
    static const char* s_previewShaderPath = "Shaders/Readback/Preview.azshader";

    static const char* s_readbackImageName = "ReadbackImage";
    static const char* s_previewImageName = "PreviewImage";

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

//        AZ::Render::Bootstrap::DefaultWindowNotificationBus::Handler::BusConnect();
//
//        ActivatePipeline();
//        CreatePasses();
    }

    void ROS2CameraSensorComponent::ActivatePipeline()
    {
        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", " Activateing pipeline.\n");
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));
        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", " Scene %p.\n", m_scene);

        // Create the pipeline
        CreatePipeline();
//
//        // Setup the pipeline
//        m_originalPipeline = m_scene->GetDefaultRenderPipeline();
//        m_scene->AddRenderPipeline(m_readbackPipeline);
//        m_scene->RemoveRenderPipeline(m_originalPipeline->GetId());
//        m_scene->SetDefaultRenderPipeline(m_readbackPipeline->GetId());
//
//        // Create an ImGuiActiveContextScope to ensure the ImGui context on the new pipeline's ImGui pass is activated.
//        m_imguiScope = AZ::Render::ImGuiActiveContextScope::FromPass({ m_readbackPipeline->GetId().GetCStr(), "ImGuiPass" });
    }

    void ROS2CameraSensorComponent::CreatePipeline()
    {
        // Create the pipeline shell
        AZ::RPI::RenderPipelineDescriptor readbackPipelineDesc;
        readbackPipelineDesc.m_mainViewTagName = "MainCamera";
        readbackPipelineDesc.m_name = "ReadbackPipeline";
        readbackPipelineDesc.m_rootPassTemplate = s_readbackPipelineTemplate;


        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", " Window %p.\n", m_windowContext.get());

        m_readbackPipeline = AZ::RPI::RenderPipeline::CreateRenderPipelineForWindow(readbackPipelineDesc, *m_windowContext);
    }

    void ROS2CameraSensorComponent::CreatePasses()
    {
        DestroyPasses();

        CreateResources();

        CreateFillerPass();
        CreatePreviewPass();

        // Add the filler and preview passes
        AZ::RPI::Ptr<AZ::RPI::ParentPass> rootPass = m_readbackPipeline->GetRootPass();
        rootPass->InsertChild(m_fillerPass, AZ::RPI::ParentPass::ChildPassIndex(0));
        rootPass->InsertChild(m_previewPass, AZ::RPI::ParentPass::ChildPassIndex(1));
    }

    void ROS2CameraSensorComponent::DestroyPasses()
    {
        if (!m_fillerPass)
        {
            return;
        }

        m_fillerPass->QueueForRemoval();
        m_fillerPass = nullptr;

        m_previewPass->QueueForRemoval();
        m_previewPass = nullptr;
    }

    void ROS2CameraSensorComponent::CreateFillerPass()
    {
        // Load the shader
        AZ::Data::AssetId shaderAssetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                shaderAssetId, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
                s_fillerShaderPath, azrtti_typeid<AZ::RPI::ShaderAsset>(), false);
        if (!shaderAssetId.IsValid())
        {
            AZ_Assert(false, "[DisplayMapperPass] Unable to obtain asset id for %s.", s_fillerShaderPath);
        }

        // Create the compute filler pass
        AZ::RPI::PassRequest createPassRequest;
        createPassRequest.m_templateName = AZ::Name(s_fillerPassTemplate);
        createPassRequest.m_passName = AZ::Name("RenderTargetPass");

        // Fill the pass data
        AZStd::shared_ptr<AZ::RPI::FullscreenTrianglePassData> passData = AZStd::make_shared<AZ::RPI::FullscreenTrianglePassData>();
        passData->m_shaderAsset.m_assetId = shaderAssetId;
        passData->m_shaderAsset.m_filePath = s_fillerShaderPath;
        createPassRequest.m_passData = AZStd::move(passData);

        // Create the connection for the output slot
        AZ::RPI::PassConnection connection = { AZ::Name("Output"), {AZ::Name("This"), AZ::Name(s_readbackImageName)} };
        createPassRequest.m_connections.push_back(connection);

        // Register the imported attachment
        AZ::RPI::PassImageAttachmentDesc imageAttachment;
        imageAttachment.m_name = s_readbackImageName;
        imageAttachment.m_lifetime = AZ::RHI::AttachmentLifetimeType::Imported;
        imageAttachment.m_assetRef.m_assetId = m_readbackImage->GetAssetId();
        createPassRequest.m_imageAttachmentOverrides.push_back(imageAttachment);

        // Create the pass
        m_fillerPass = AZ::RPI::PassSystemInterface::Get()->CreatePassFromRequest(&createPassRequest);
    }

    void ROS2CameraSensorComponent::CreatePreviewPass()
    {
        // Load the shader
        AZ::Data::AssetId shaderAssetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                shaderAssetId, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
                s_previewShaderPath, azrtti_typeid<AZ::RPI::ShaderAsset>(), false);
        if (!shaderAssetId.IsValid())
        {
            AZ_Assert(false, "[DisplayMapperPass] Unable to obtain asset id for %s.", s_previewShaderPath);
        }

        // Create the compute filler pass
        AZ::RPI::PassRequest createPassRequest;
        createPassRequest.m_templateName = AZ::Name(s_previewPassTemplate);
        createPassRequest.m_passName = AZ::Name("PreviewPass");

        AZStd::shared_ptr<AZ::RPI::FullscreenTrianglePassData> passData = AZStd::make_shared<AZ::RPI::FullscreenTrianglePassData>();
        passData->m_shaderAsset.m_assetId = shaderAssetId;
        passData->m_shaderAsset.m_filePath = s_previewShaderPath;
        createPassRequest.m_passData = AZStd::move(passData);

        // Create the connection for the output slot
        AZ::RPI::PassConnection outputConnection = { AZ::Name("Output"), {AZ::Name("Parent"), AZ::Name("SwapChainOutput")} };
        createPassRequest.m_connections.push_back(outputConnection);
        AZ::RPI::PassConnection inputConnection = { AZ::Name("Input"), {AZ::Name("This"), AZ::Name(s_previewImageName)} };
        createPassRequest.m_connections.push_back(inputConnection);

        // Register the imported attachment
        AZ::RPI::PassImageAttachmentDesc imageAttachment;
        imageAttachment.m_name = s_previewImageName;
        imageAttachment.m_lifetime = AZ::RHI::AttachmentLifetimeType::Imported;
        imageAttachment.m_assetRef.m_assetId = m_previewImage->GetAssetId();
        createPassRequest.m_imageAttachmentOverrides.push_back(imageAttachment);

        m_previewPass = AZ::RPI::PassSystemInterface::Get()->CreatePassFromRequest(&createPassRequest);
    }

    void ROS2CameraSensorComponent::CreateResources()
    {
        AZ::Data::Instance<AZ::RPI::AttachmentImagePool> pool = AZ::RPI::ImageSystemInterface::Get()->GetSystemAttachmentPool();

        // Create the readback target
        {
            AZ::RPI::CreateAttachmentImageRequest createRequest;
            createRequest.m_imageName = AZ::Name(s_readbackImageName);
            createRequest.m_isUniqueName = false;
            createRequest.m_imagePool = pool.get();
            createRequest.m_imageDescriptor = AZ::RHI::ImageDescriptor::Create2D(AZ::RHI::ImageBindFlags::Color | AZ::RHI::ImageBindFlags::ShaderWrite | AZ::RHI::ImageBindFlags::CopyRead | AZ::RHI::ImageBindFlags::CopyWrite, m_resourceWidth, m_resourceHeight, AZ::RHI::Format::R8G8B8A8_UNORM);

            m_readbackImage = AZ::RPI::AttachmentImage::Create(createRequest);
        }

        // Create the preview image
        {
            AZ::RPI::CreateAttachmentImageRequest createRequest;
            createRequest.m_imageName = AZ::Name(s_previewImageName);
            createRequest.m_isUniqueName = false;
            createRequest.m_imagePool = pool.get();
            createRequest.m_imageDescriptor = AZ::RHI::ImageDescriptor::Create2D(AZ::RHI::ImageBindFlags::ShaderRead | AZ::RHI::ImageBindFlags::CopyRead | AZ::RHI::ImageBindFlags::CopyWrite, m_resourceWidth, m_resourceHeight, AZ::RHI::Format::R8G8B8A8_UNORM);

            m_previewImage = AZ::RPI::AttachmentImage::Create(createRequest);
        }
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
//        DestroyPasses();
//        DeactivatePipeline();
//
//        AZ::Render::Bootstrap::DefaultWindowNotificationBus::Handler::BusDisconnect();
//
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::DeactivatePipeline()
    {
        m_imguiScope = {}; // restores previous ImGui context.

        m_scene->AddRenderPipeline(m_originalPipeline);
        m_scene->RemoveRenderPipeline(m_readbackPipeline->GetId());

        m_readbackPipeline = nullptr;
    }

    void ROS2CameraSensorComponent::DefaultWindowCreated()
    {

        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", " Window created.\n");
        AZ::Render::Bootstrap::DefaultWindowBus::BroadcastResult(m_windowContext, &AZ::Render::Bootstrap::DefaultWindowBus::Events::GetDefaultWindowContext);
        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", " Window created to %p.\n", m_windowContext.get());
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
//        UpdateCamera();
//        // Readback was completed, we need to update the preview image
//        if (m_textureNeedsUpdate)
//        {
//            UploadReadbackResult();
//
//            AZ_Error("ReadbackExample", m_resourceWidth == m_readbackStat.m_descriptor.m_size.m_width, "Incorrect resource width read back.");
//            AZ_Error("ReadbackExample", m_resourceHeight == m_readbackStat.m_descriptor.m_size.m_height, "Incorrect resource height read back.");
//
//            m_textureNeedsUpdate = false;
//        }
//
//        static int frame_id = 0;
//
//        std::stringstream ss;
//        ss << "/home/pzyskowski/tmp/o3de_capture/capture" << frame_id++ << ".ppm";
//        std::string filename = ss.str();
//        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", "saving %s.\n", filename.c_str());

        static int count = 0;

        if (count++ == 10) {
            AZ::RPI::PassSystemInterface::Get()->GetRootPass()->DebugPrint();
        }

        AZ::Render::FrameCaptureRequestBus::Broadcast(
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
//                AZStd::vector<AZStd::string>(1, "Root.MainPipeline_0.DebugOverlayPass"),
//                AZStd::vector<AZStd::string>(1, "DebugOverlayPass"),
//                AZStd::string("InputOutput"),
                AZStd::vector<AZStd::string>(1, "CopyToSwapChain"),
                AZStd::string("Output"),
                AZStd::bind(&ROS2CameraSensorComponent::ReadbackCallback, this, AZStd::placeholders::_1),
                AZ::RPI::PassAttachmentReadbackOption::Output);

    }

    void ROS2CameraSensorComponent::UploadReadbackResult() const
    {
        const AZ::RHI::ImageSubresourceRange range(0, 0, 0, 0);
        AZ::RHI::ImageSubresourceLayoutPlaced layout;
        m_previewImage->GetRHIImage()->GetSubresourceLayouts(range, &layout, nullptr);
        AZ::RHI::ImageUpdateRequest updateRequest;
        updateRequest.m_image = m_previewImage->GetRHIImage();
        updateRequest.m_sourceSubresourceLayout = layout;
        updateRequest.m_sourceData = m_resultData->begin();
        updateRequest.m_imageSubresourcePixelOffset = AZ::RHI::Origin(0, 0, 0);
        m_previewImage->UpdateImageContents(updateRequest);
    }

    void ROS2CameraSensorComponent::UpdateCamera() {
//        PassesChanged();
        PerformReadback();
    }

    void ROS2CameraSensorComponent::PassesChanged()
    {
        DestroyPasses();
        CreatePasses();
    }

    void ROS2CameraSensorComponent::PerformReadback()
    {
        AZ_Assert(m_fillerPass, "Render target pass is null.");

        if (!m_readback)
        {
            m_readback = AZStd::make_shared<AZ::RPI::AttachmentReadback>(AZ::RHI::ScopeId{ "RenderTargetCapture" });
            m_readback->SetCallback(AZStd::bind(&ROS2CameraSensorComponent::ReadbackCallback, this, AZStd::placeholders::_1));
        }

        m_fillerPass->ReadbackAttachment(m_readback, AZ::Name("Output"));
    }

    void ROS2CameraSensorComponent::ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", "Received radback data.\n");

        AZ::RHI::ImageDescriptor descriptor = result.m_imageDescriptor;

        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", "Image descriptor: %s.\n", result.m_name.GetCStr());
        if (result.m_dataBuffer) {
            AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", "array size: %u\n",
                           result.m_dataBuffer->size());
        }

        AZ_TracePrintf(">>>>>>>>>>>> ROS2CameraComponent", "size: %dx%dx%d format : %u\n",
                       descriptor.m_size.m_width,
                       descriptor.m_size.m_height,
                       descriptor.m_size.m_depth,
                       result.m_imageDescriptor.m_format);

        sensor_msgs::msg::Image message;
        message.encoding = sensor_msgs::image_encodings::BGRA8;

        message.width = descriptor.m_size.m_width;
        message.height = descriptor.m_size.m_height;

        message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());

//        for (int i = 0; i < message.width * message.height; i++) {
//            size_t idx = i * numChannels;
//            message.data[idx] = 0;
//            message.data[idx + 1] = i <= 50 ? 255 : 0;
//            message.data[idx + 2] = i >= 50 ? 255 : 0;
//        }
        message.header.frame_id = "camera";

        m_imagePublisher->publish(message);
    }

    void ROS2CameraSensorComponent::OnTransformChanged(const AZ::Transform&, const AZ::Transform&)
    {
        const AZ::EntityId* currentBusId = AZ::TransformNotificationBus::GetCurrentBusId();
        AZ::Render::DirectionalLightFeatureProcessorInterface* directionalLightFeatureProcessor = AZ::RPI::Scene::GetFeatureProcessorForEntityContextId<AZ::Render::DirectionalLightFeatureProcessorInterface>(m_entityContextId);
        if (currentBusId && *currentBusId == m_cameraEntityId && directionalLightFeatureProcessor)
        {
            auto transform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(
                    transform,
                    m_cameraEntityId,
                    &AZ::TransformBus::Events::GetWorldTM);
            for (const AZ::Render::DirectionalLightFeatureProcessorInterface::LightHandle& handle : m_lightHandles)
            {
                directionalLightFeatureProcessor->SetCameraTransform(handle, transform);
            }
        }
    }
} // namespace ROS2
