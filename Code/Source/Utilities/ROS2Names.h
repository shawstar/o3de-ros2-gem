/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Utility class for handling ROS2 naming rules.
    class ROS2Names
    {
    public:
        //! Joins namespace and the given name.
        static AZStd::string GetNamespacedName(const AZStd::string& ns, const AZStd::string& name);

        //! Converts input to a ROS2-acceptable name for topics and namespaces.
        //! Any characters not fitting ROS2 naming specification are replaced with underscores.
        static AZStd::string RosifyName(const AZStd::string& input);

        //! Validates namespace adherence to ROS2 specification. Delegates validation to ROS2 layers.
        static AZ::Outcome<void, AZStd::string> ValidateNamespace(const AZStd::string& ros2Namespace);

        //! Validate namespace field. Fits ChangeValidate for Editor fields.
        static AZ::Outcome<void, AZStd::string> ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType);

        //! Validate topic adherence to ROS2 specification.
        static AZ::Outcome<void, AZStd::string> ValidateTopic(const AZStd::string& topic);

        //! Validate topic field. Fits ChangeValidate for Editor fields.
        static AZ::Outcome<void, AZStd::string> ValidateTopicField(void* newValue, const AZ::Uuid& valueType);
    };
}  // namespace ROS2
