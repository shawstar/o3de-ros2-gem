/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Conversions.h"

namespace ROS2
{
    AZ::Vector3 ROS2Conversions::FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector)
    {   // Same coordinate systems - just translate types
        return AZ::Vector3(ros2vector.x, ros2vector.y, ros2vector.z);
    }

    geometry_msgs::msg::Vector3 ROS2Conversions::ToROS2Vector3(const AZ::Vector3& azvector)
    {
        geometry_msgs::msg::Vector3 ros2vector;
        ros2vector.x = azvector.GetX();
        ros2vector.y = azvector.GetY();
        ros2vector.z = azvector.GetZ();
        return ros2vector;
    }

    geometry_msgs::msg::Quaternion ROS2Conversions::ToROS2Quaternion(const AZ::Quaternion& azquaternion)
    {
        geometry_msgs::msg::Quaternion ros2Quaternion;
        ros2Quaternion.x = azquaternion.GetX();
        ros2Quaternion.y = azquaternion.GetY();
        ros2Quaternion.z = azquaternion.GetZ();
        ros2Quaternion.w = azquaternion.GetW();
        return ros2Quaternion;
    }
}  // namespace ROS2
