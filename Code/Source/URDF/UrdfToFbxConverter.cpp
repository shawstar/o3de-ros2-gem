/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UrdfToFbxConverter.h"

#include <AzCore/std/string/string.h>

#include <AzCore/Console/Console.h>

namespace ROS2
{
    std::string UrdfToFbxConverter::Convert(const std::string & urdfString)
    {
        // 1. Parse URDF
        const auto urdf = UrdfParser::Parse(urdfString);

        // 2. Add materials to FBX
        AddMaterialsToFbxGenerator(urdf);

        // 3. Add objects from URDF based structure to FBX generator
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
            const auto cubeId = m_generator.AddCubeObject(linkName, cubeSize, m_materialNamesToIds["black"]);
            (void)cubeId;
        }
        else
        {
            AZ_Warning(__func__, false, "Only box geometry is supported.");
        }

        // TODO: handle joints

        return m_generator.GetFbxString();
    }

    std::string UrdfToFbxConverter::ConvertAndSaveToFile(
        const std::string & urdfString, const std::string & filePath)
    {
        const auto fbxContent = Convert(urdfString);
        m_generator.SaveToFile(filePath);

        return fbxContent;
    }

    void UrdfToFbxConverter::AddMaterialsToFbxGenerator(const urdf::ModelInterfaceSharedPtr & urdfModel)
    {
        if (!urdfModel)
        {
            AZ_Error(__func__, false, "Missing URDF model.");
            return;
        }

        for (const auto & e : urdfModel->materials_)
        {
            const auto material = e.second;
            const std::string & materialName = material->name;
            const auto materialColor = material->color;
            const Fbx::Color fbxColor(materialColor.r, materialColor.g, materialColor.b);
            const auto materialId = m_generator.AddMaterial(materialName, fbxColor);
            m_materialNamesToIds[materialName] = materialId;

            AZ_Printf(__func__, "Add new material: %s", materialName.c_str());
        }
    }

} // namespace ROS2