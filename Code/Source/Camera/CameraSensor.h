#pragma once

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <chrono>

class Entity;

namespace ROS2 {

    struct CameraSensorDescription {
        float m_VerticalFieldOfViewDeg = 90.0f;
        int m_width = 640;
        int m_height = 480;
        float m_aspectRatio = 1.0f;
        float m_nearDist = 0.1f;
        float m_farDist = 100.0f;
        AZStd::string cameraName = "camera";

        AZ::Matrix4x4 MakeViewToClipMatrix();
    };

    class CameraSensor {
    public:
        void InitializeSecondPipeline(AZ::EntityId entityId);
        void DestroyPipeline();
        void RequestFrame(const AZ::Transform& cameraPose, std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);
        void SetCameraDescription(const CameraSensorDescription& description);

    private:
        void InitializeView();
        float SecondsSinceStart();
        std::chrono::steady_clock::time_point m_startTime;

        CameraSensorDescription m_cameraSensorDescription;

        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::ViewPtr m_targetView;

        AZ::RPI::Scene* m_scene = nullptr;
    };

}
