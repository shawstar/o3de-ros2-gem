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

#include <AzCore/Component/Component.h>

#include "Sensor/ROS2SensorComponent.h"

#include <Atom/RPI.Public/DynamicDraw/DynamicDrawContext.h>
#include <Atom/RPI.Public/Image/AttachmentImage.h>
#include <Atom/RPI.Public/Pass/Specific/ImageAttachmentPreviewPass.h>
#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <Atom/RPI.Public/WindowContext.h>
#include <Atom/Bootstrap/DefaultWindowBus.h>
#include <AzFramework/Entity/EntityContextBus.h>

#include <AzCore/Component/TransformBus.h>

#include <Atom/Feature/Mesh/MeshFeatureProcessor.h>
#include <Atom/Feature/ImGui/ImGuiUtils.h>
#include <Atom/Feature/Mesh/MeshFeatureProcessorInterface.h>
#include <Atom/Feature/Utils/LightingPreset.h>
#include <Atom/Feature/SkyBox/SkyBoxFeatureProcessorInterface.h>
#include <Atom/Feature/PostProcess/PostProcessFeatureProcessorInterface.h>
#include <Atom/Feature/ImageBasedLights/ImageBasedLightFeatureProcessorInterface.h>
#include <Atom/Feature/CoreLights/DirectionalLightFeatureProcessorInterface.h>
#include <Atom/Utils/AssetCollectionAsyncLoader.h>

namespace ROS2
{
    class ROS2CameraSensorComponent
        : public ROS2SensorComponent
        , public AZ::Render::Bootstrap::DefaultWindowNotificationBus::Handler
        , public AZ::TransformNotificationBus::MultiHandler
    {
    public:
        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", ROS2SensorComponent);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

        // ========================= SampleComponentBase ============================
    public:
//        //! Init and shut down should be called in derived components' Activate() and Deactivate().
//        //! @param loadDefaultLightingPresets if true, it will scan all lighting presets in the project and load them.
//        void InitLightingPresets(bool loadDefaultLightingPresets = false);
//        void ShutdownLightingPresets();
//
//        //! Add a drop down list to select lighting preset for this sample.
//        //! Lighting presets must be loaded before calling this function, otherwise the list will be hide.
//        //! It should be called between ImGui::Begin() and ImGui::End().
//        //! e.g. Calling it between ImGuiSidebar::Begin() and ImGuiSidebar::End() will embed this list into the side bar.
//        void ImGuiLightingPreset();
//
//        //! Load lighting presets from an asset.
//        //! It will clear any presets loaded previously.
//        void LoadLightingPresetsFromAsset(const AZStd::string& assetPath);
//
//        //! Load lighting presets from an asset.
//        //! Append the presets to the current existing presets.
//        void AppendLightingPresetsFromAsset(const AZStd::string& assetPath);
//
//        //! Clear all lighting presets.
//        void ClearLightingPresets();
//
//        //! Reset internal scene related data
//        void ResetScene();
//
//        //! Apply lighting presets to the scene.
//        //! Derived samples can override this function to have custom behaviors.
//        virtual void OnLightingPresetSelected(const AZ::Render::LightingPreset& preset, bool useAltSkybox);
//
//        //! Return the AtomSampleViewer EntityContextId, retrieved from the ComponentConfig
//        AzFramework::EntityContextId GetEntityContextId() const;
//
//        //! Return the AtomSampleViewer camera EntityId, retrieved from the ComponentConfig
//        AZ::EntityId GetCameraEntityId() const;
//
//        AZ::Render::MeshFeatureProcessorInterface* GetMeshFeatureProcessor() const;
//
//        void OnLightingPresetEntityShutdown(const AZ::EntityId& entityId);
//
//        // Preload assets
//        void PreloadAssets(const AZStd::vector<AZ::AssetCollectionAsyncLoader::AssetToLoadInfo>& assetList);

        //! Async asset load
        AZ::AssetCollectionAsyncLoader m_assetLoadManager;

        // The callback might be called more than one time if there are more than one asset are ready in one frame.
        // Use this flag to prevent OnAllAssetsReadyActivate be called more than one time.
        bool m_isAllAssetsReady = false;

        AZStd::string m_sampleName;

        AZ::RPI::Scene* m_scene = nullptr;

    private:
        // AZ::TransformNotificationBus::MultiHandler overrides...
        void OnTransformChanged(const AZ::Transform&, const AZ::Transform&) override;

        // virtual call back function which is called when all preloaded assets are loaded.
        virtual void OnAllAssetsReadyActivate() {};

        AzFramework::EntityContextId m_entityContextId;
        AZ::EntityId m_cameraEntityId;
        mutable AZ::Render::MeshFeatureProcessorInterface* m_meshFeatureProcessor = nullptr;

        //! All loaded lighting presets.
        struct LightingPresetEntry
        {
            AZStd::string m_displayName;
            AZ::Render::LightingPreset m_preset;
        };
        AZStd::vector<LightingPresetEntry> m_lightingPresets;

        //! Lights created by lighting presets.
        AZStd::vector<AZ::Render::DirectionalLightFeatureProcessorInterface::LightHandle> m_lightHandles;

        //! Post process entity to handle ExposureControlSettings.
        AZ::Entity* m_postProcessEntity = nullptr;

        //! Dirty flag is set to true when m_lightingPresets is modified.
        bool m_lightingPresetsDirty = true;

        //! Current active lighting preset.
        constexpr static int32_t InvalidLightingPresetIndex = -1;
        int32_t m_currentLightingPresetIndex = InvalidLightingPresetIndex;
        bool m_useAlternateSkybox = false; //!< LightingPresets have an alternate skybox that can be used, when this is true. This is usually a blurred version of the primary skybox.
        // ========================= END SampleComponentBase ============================


        // ========================= ReadbackComponent ============================
    private:

        void FrequencyTick() override;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        AZStd::string m_cameraName = "dummy";

        void DefaultWindowCreated() override;

        void CreatePipeline();
        void ActivatePipeline();
        void DeactivatePipeline();

        void CreatePasses();
        void DestroyPasses();
        void PassesChanged();

        void CreateFillerPass();
        void CreatePreviewPass();

        void CreateResources();

        void PerformReadback();
        void ReadbackCallback(const AZ::RPI::AttachmentReadback::ReadbackResult& result);
        void UploadReadbackResult() const;

        void UpdateCamera();

        // Pass used to render the pattern and support the readback operation
        AZ::RHI::Ptr<AZ::RPI::Pass> m_fillerPass;
        // Pass used to display the readback result back on the screen
        AZ::RHI::Ptr<AZ::RPI::Pass> m_previewPass;

        // Image used as the readback source
        AZ::Data::Instance<AZ::RPI::AttachmentImage> m_readbackImage;
        // Image used as the readback result destination
        AZ::Data::Instance<AZ::RPI::AttachmentImage> m_previewImage;

        // Custom pipeline
        AZStd::shared_ptr<AZ::RPI::WindowContext> m_windowContext;
        AZ::RPI::RenderPipelinePtr m_readbackPipeline;
        AZ::RPI::RenderPipelinePtr m_originalPipeline;
        AZ::Render::ImGuiActiveContextScope m_imguiScope;

        // Readback
        AZStd::shared_ptr<AZ::RPI::AttachmentReadback> m_readback;
        // Holder for the host available copy of the readback data
        AZStd::shared_ptr<AZStd::vector<uint8_t>> m_resultData;
        struct {
            AZ::Name m_name;
            size_t m_bytesRead;
            AZ::RHI::ImageDescriptor m_descriptor;
        } m_readbackStat;
        bool m_textureNeedsUpdate = false;

        uint32_t m_resourceWidth = 512;
        uint32_t m_resourceHeight = 512;
        // ========================= END ReadbackComponent ============================


    };
}  // namespace ROS2
