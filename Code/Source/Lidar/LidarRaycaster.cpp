/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "LidarRaycaster.h"
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace ROS2
{
    // A simplified, non-optimized first version. TODO - generalize results (fields)
    AZStd::vector<AZ::Vector3> LidarRaycaster::PerformRaycast(const AZ::Vector3& start, const AZStd::vector<AZ::Vector3>& directions, float distance)
    {
        AZStd::vector<AZ::Vector3> results;
        AzPhysics::SceneQueryRequests requests;
        requests.reserve(directions.size());
        for (const AZ::Vector3& direction : directions)
        {   // NOTE - performance-wise, consider reusing requests
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = start;
            request->m_direction = direction;
            request->m_distance = distance;
            request->m_reportMultipleHits = false;
            requests.emplace_back(AZStd::move(request));
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        if (sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            AZ_Warning("LidarRaycaster", false, "No valid scene handle");
            return results;
        }
        auto requestResults = sceneInterface->QuerySceneBatch(sceneHandle, requests);
        for (const auto& requestResult : requestResults)
        {   // TODO - check flag for SceneQuery::ResultFlags::Position
            if (requestResult.m_hits.size() > 0)
            {
                results.push_back(requestResult.m_hits[0].m_position);
            }
        }
        return results;
    }
} // namespace ROS2