/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>

#include "FbxGenerator.h"
#include "UrdfParser.h"

namespace ROS2
{
    //! Class for conversion from URDF to Filmbox (.fbx) files
    class UrdfToFbxConverter
    {
    public:
        AZStd::string ConvertUrdfToFbx(const AZStd::string & urdfString);

    private:
        void AddMaterialsToFbxGenerator(const urdf::ModelInterfaceSharedPtr & urdfModel);

        Fbx::FbxGenerator m_generator;
        std::map <std::string, Id> m_materialNamesToIds;
    };

} // namespace ROS2