/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <URDF/UrdfToFbxConverter.h>

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

namespace UnitTest
{

class UrdfToFbxConverterTest : public AllocatorsTestFixture
{
    public:
        AZStd::string GetUrdfWithOneLink()
        {
            return
                "<robot name=\"test_one_link\">"
                "  <material name=\"black\">"
                "    <color rgba=\"0.0 0.0 0.0 1.0\"/>"
                "  </material>"
                "  <link name=\"link1\">"
                "    <inertial>"
                "      <mass value=\"1.0\"/>"
                "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                "    </inertial>"
                "    <visual>"
                "      <geometry>"
                "        <box size=\"1.0 2.0 1.0\"/>"
                "      </geometry>"
                "      <material name=\"black\"/>"
                "    </visual>"
                "    <collision>"
                "      <geometry>"
                "        <box size=\"1.0 2.0 1.0\"/>"
                "      </geometry>"
                "    </collision>"
                "  </link>"
                "</robot>";
        }

    void PrintFbxContent(const std::string & str)
    {
        std::cout << __func__ << " fbx data:"
            << "\n---------------\n"
            << str
            << "\n---------------\n";
    }
};

TEST_F(UrdfToFbxConverterTest, ConvertUrdfWithOneLink)
{
    ROS2::UrdfToFbxConverter converter;
    const auto xmlStr = GetUrdfWithOneLink();
    const auto fbxStr = converter.ConvertUrdfToFbx(xmlStr);

    PrintFbxContent(fbxStr);
    // Add validation (implementation of converter is also not ready)
}

} // namespace
