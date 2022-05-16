/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UrdfToFbxConverter.h"

#include <AzCore/std/string/string.h>

#include "UrdfParser.h"
#include "FbxGenerator.h"

namespace ROS2
{
    AZStd::string UrdfToFbxConverter::ConvertUrdfToFbx(const AZStd::string & urdfString)
    {
        // TODO: Add implementation

        // 1. Parse URDF with UrdfParser
        UrdfParser parser;
        const auto urdf = parser.Parse(urdfString);

        // 2. Create FBX file with FbxGenerator
        Fbx::FbxGenerator generator;

        // Add material
        // TODO: add more materials
        Fbx::Color color(0.0, 0.0, 0.0);
        const auto materialId = generator.AddMaterial("black", color);

        // 3. Add each object from URDF based structure to FBX generator
        const auto root = urdf->getRoot();
        
        // TODO: Add depth first search - now it works only for depth = 1
        // while (const auto link = root; !link->child_links.empty())
        // {
        // }
        // Get link from URDF and add to FBX
        const auto link = root; // TODO: just PoC
        const auto linkName = link->name;
        const auto linkGeometry = link->visual->geometry;
        // TODO: handle different materials, link.visual.material

        if (linkGeometry->type == urdf::Geometry::BOX)
        {
            auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(linkGeometry);
            const double cubeSize = boxGeometry->dim.x; // TODO: Handle box in FBX instead of cube
            const auto cubeId = generator.AddCubeObject(linkName, cubeSize, materialId);
            (void)cubeId;
        }
        else
        {
            // Only box is supported now
        }

        // TODO: handle joints

        return generator.GetFbxString();
    }

} // namespace ROS2