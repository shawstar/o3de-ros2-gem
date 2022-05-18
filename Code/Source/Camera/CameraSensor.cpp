#include "CameraSensor.h"

#include <AzCore/Math/MatrixUtils.h>

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

namespace ROS2 {

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() {
        m_aspectRatio = static_cast<float>(m_width)/static_cast<float>(m_height);

        AZ::Matrix4x4 viewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(viewToClipMatrix, AZ::DegToRad(m_VerticalFieldOfViewDeg), m_aspectRatio, m_nearDist, m_farDist, true);
        return viewToClipMatrix;
    }

    const char* const cameraSensorTracePrefix = "CameraSensor";

    void CameraSensor::SetCameraDescription(const CameraSensorDescription& description) {
        m_cameraSensorDescription = description;
    }

    void CameraSensor::InitializeSecondPipeline(AZ::EntityId entityId) {
        AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(m_cameraSensorDescription.MakeViewToClipMatrix());
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        AZStd::string pipelineName = m_cameraSensorDescription.cameraName + "Pipeline";

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";          //Surface shaders render to the "MainCamera" tag
        pipelineDesc.m_name = pipelineName;                 //Sets the debug name for this pipeline
        pipelineDesc.m_rootPassTemplate = "MainPipelineRenderToTexture";    //References a template in AtomSampleViewer\Passes\PassTemplates.azasset
        pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(m_cameraSensorDescription.m_width, m_cameraSensorDescription.m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        m_passHierarchy.push_back(pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        m_pipeline->SetDefaultView(m_view);
        m_targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            // This will be set again to mimic the active camera in UpdateView
            fp->SetViewAlias(m_view, m_targetView);
        }

        m_startTime = std::chrono::steady_clock::now();
    }

    void CameraSensor::DestroyPipeline()
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

    void CameraSensor::RequestFrame(const AZ::Transform& cameraPose, std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback) {

        AZ::Vector3 eulerAngles = AZ::ConvertTransformToEulerDegrees(cameraPose);
        AZ::Vector3 translation = cameraPose.GetTranslation();

        AZ_TracePrintf(cameraSensorTracePrefix, "Translation: %f, %f, %f, euler: %f, %f, %f",
                       translation.GetX(), translation.GetY(), translation.GetZ(),
                       eulerAngles.GetX(), eulerAngles.GetY(), eulerAngles.GetZ());

        AZ::Transform inverse = cameraPose.GetInverse();

        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(),
                                                                                       inverse.GetTranslation()));

        AZ::Render::FrameCaptureRequestBus::Broadcast(
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
                m_passHierarchy,
                AZStd::string("Output"),
                callback,
                AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    float CameraSensor::SecondsSinceStart() {
        return static_cast<float>(
                       std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - m_startTime).count())/1000.0f;
    }
}
