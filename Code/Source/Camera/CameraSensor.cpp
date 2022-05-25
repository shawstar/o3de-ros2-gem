#include "CameraSensor.h"

#include <AzCore/Math/MatrixUtils.h>

#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Base.h>

#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>

#include <PostProcess/PostProcessFeatureProcessor.h>

#include <Camera/CameraCaptureWrapper.h>

namespace ROS2 {

    const char* const cameraSensorTracePrefix = "CameraSensor";

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() {
        m_aspectRatio = static_cast<float>(m_width)/static_cast<float>(m_height);

        AZ::Matrix4x4 viewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(viewToClipMatrix, AZ::DegToRad(m_VerticalFieldOfViewDeg), m_aspectRatio, m_nearDist, m_farDist, true);
        return viewToClipMatrix;
    }

    void CameraSensor::SetCameraDescription(const CameraSensorDescription& description) {
        m_cameraSensorDescription = description;
    }

    void CameraSensor::InitializeSecondPipeline() {
        AZ_TracePrintf(cameraSensorTracePrefix, "Initializing pipeline for %s", m_cameraSensorDescription.cameraName.c_str());
        AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(m_cameraSensorDescription.MakeViewToClipMatrix());
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        AZStd::string pipelineName = m_cameraSensorDescription.cameraName + "Pipeline";

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = pipelineName;
        pipelineDesc.m_rootPassTemplate = "MainPipelineRenderToTexture";
        pipelineDesc.m_renderSettings.m_multisampleState.m_samples = 4;
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
        m_pipeline->RemoveFromRenderTick();

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(m_cameraSensorDescription.m_width, m_cameraSensorDescription.m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        m_passHierarchy.push_back(pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        m_pipeline->SetDefaultView(m_view);
        m_targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>()) {
            fp->SetViewAlias(m_view, m_targetView);
        }
    }

    void CameraSensor::DestroyPipeline() {
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            fp->RemoveViewAlias(m_view);
        }
        m_scene->RemoveRenderPipeline(m_pipeline->GetId());
        m_passHierarchy.clear();
        m_pipeline.reset();
        m_view.reset();
        m_targetView.reset();
    }

    void CameraSensor::RequestFrame(const AZ::Transform& cameraPose, std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback) {
        AZ::Transform inverse = cameraPose.GetInverse();

        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(),
                                                                                       inverse.GetTranslation()));

        CameraCaptureWrapper::Get().RequestFrame(m_passHierarchy, std::move(callback), m_pipeline);
    }
}
