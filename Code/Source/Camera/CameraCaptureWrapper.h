#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>

#include <Clock/SimulationClock.h>

#include <Atom/RPI.Public/RenderPipeline.h>

#include <list>

class CameraCaptureWrapper : public AZ::Render::FrameCaptureNotificationBus::Handler
{
public:
    static CameraCaptureWrapper& Get() {
        static CameraCaptureWrapper i;
        return i;
    }

    void RequestFrame(AZStd::vector<AZStd::string> passHierarchy,
                      std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> frameRenderedCallback,
                      AZ::RPI::RenderPipelinePtr pipeline ) {
        AZ_TracePrintf("CameraCaptureWrapper", "Requesting frame capture %s\n", passHierarchy[0].c_str());
        CameraCaptureRequest request(passHierarchy, frameRenderedCallback, pipeline);
        auto found_it = std::find(captureRequestQueue.cbegin(), captureRequestQueue.cend(), request);
                       if (found_it != captureRequestQueue.cend()) {
            AZ_TracePrintf("CameraCaptureWrapper",
                           "Request to render in %s already in progress: %u\n", request.pipelineHierarchy[0].c_str(),
                                                                                captureRequestQueue.size());
            return;
        }

        AZ_TracePrintf("CameraCaptureWrapper", "New request %s\n", request.pipelineHierarchy[0].c_str());
        captureRequestQueue.emplace_back(std::move(request));
        AZ_TracePrintf("CameraCaptureWrapper", "Queue size %u\n", captureRequestQueue.size());

        if (captureInProgres) {
            AZ_TracePrintf("CameraCaptureWrapper", "Request to render already in progress: %u\n");
            return;
        }

        auto callback = [this](const AZ::RPI::AttachmentReadback::ReadbackResult& result) {
            auto currentRequest = captureRequestQueue.begin();
            AZ_TracePrintf("CameraCaptureWrapper", "Frame done %s\n", currentRequest->pipelineHierarchy[0].c_str());
            currentRequest->frameRenderedCallback(result);
        };

        pipeline->AddToRenderTickOnce();
        AZ::Render::FrameCaptureRequestBus::Broadcast(
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
                passHierarchy,
                AZStd::string("Output"),
                callback,
                AZ::RPI::PassAttachmentReadbackOption::Output);
        captureInProgres = true;
    }


    void OnCaptureFinished(AZ::Render::FrameCaptureResult result, const AZStd::string& info) override {
        AZ_TracePrintf("CameraCaptureWrapper", "Capture done\n");
        captureInProgres = false;
        captureRequestQueue.erase(captureRequestQueue.begin());
    };

private:
    CameraCaptureWrapper() {
        AZ::Render::FrameCaptureNotificationBus::Handler::BusConnect();
    };

    ~CameraCaptureWrapper() override {
        AZ::Render::FrameCaptureNotificationBus::Handler::BusDisconnect();
    }

    struct CameraCaptureRequest {
        AZStd::vector<AZStd::string> pipelineHierarchy;
        std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> frameRenderedCallback;
        AZ::RPI::RenderPipelinePtr pipeline;

        // Compare only pipeline name
        friend bool operator==(const CameraCaptureRequest& lhs, const CameraCaptureRequest& rhs) {
            return lhs.pipelineHierarchy[0] == rhs.pipelineHierarchy[0];
        }

        CameraCaptureRequest(
                AZStd::vector<AZStd::string> pipelineHierarchy,
                std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> frameRenderedCallback,
                AZ::RPI::RenderPipelinePtr pipeline)
                : pipelineHierarchy(std::move(pipelineHierarchy))
                , frameRenderedCallback(std::move(frameRenderedCallback))
                , pipeline(std::move(pipeline)) { }

        CameraCaptureRequest(CameraCaptureRequest&& other)
                : pipelineHierarchy(std::move(other.pipelineHierarchy))
                , frameRenderedCallback(std::move(other.frameRenderedCallback))
                , pipeline(std::move(other.pipeline)) { }

        CameraCaptureRequest& operator=(const CameraCaptureRequest& other) = default;
    };

    bool captureInProgres = false;
    std::list<CameraCaptureRequest> captureRequestQueue;
};