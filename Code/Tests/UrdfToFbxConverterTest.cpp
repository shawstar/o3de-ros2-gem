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

        std::string GetUrdfWithTwoLinksAndOneJoint()
        {
            std::string xmlStr =
                "<robot name=\"test_one_link\">"
                "  <material name=\"black\">"
                "    <color rgba=\"0.0 0.0 0.0 1.0\"/>"
                "  </material>"
                "  <material name=\"blue\">"
                "    <color rgba=\"0.0 0.0 0.8 1.0\"/>"
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
                "  <link name=\"link2\">"
                "    <inertial>"
                "      <mass value=\"1.0\"/>"
                "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                "    </inertial>"
                "    <visual>"
                "      <geometry>"
                "        <box size=\"1.0 2.0 1.0\"/>"
                "      </geometry>"
                "      <material name=\"blue\"/>"
                "    </visual>"
                "    <collision>"
                "      <geometry>"
                "        <box size=\"2.0 2.0 1.0\"/>"
                "      </geometry>"
                "    </collision>"
                "  </link>"
                "  <joint name=\"joint1\" type=\"continuous\">"
                "    <parent link=\"link1\"/>"
                "    <child link=\"link2\"/>"
                "    <origin xyz=\"0.5 0.25 0.0\" rpy=\"0 0 0\"/>"
                "    <axis xyz=\"0 0 1\"/>"
                "  </joint>"
                "</robot>";
            return xmlStr;
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
    const auto urdfStr = GetUrdfWithOneLink();

    // Save generated FBX to file (it's then loaded by Asset Processor).
    std::string projectPath = "/home/mdrwiega/o3de/Ros2WarehouseDemo/one_link.fbx";
    const auto fbxStr = converter.ConvertAndSaveToFile(urdfStr, projectPath);

    PrintFbxContent(fbxStr);
}

TEST_F(UrdfToFbxConverterTest, ConvertUrdfWithTwoLinksAndJoint)
{
    const auto urdfStr = GetUrdfWithTwoLinksAndOneJoint();

    // Save generated FBX to file (it's then loaded by Asset Processor).
    std::string projectPath = "/home/mdrwiega/o3de/Ros2WarehouseDemo/two_links_one_joint.fbx";
    const auto fbxStr = converter.ConvertAndSaveToFile(urdfStr, projectPath);

    PrintFbxContent(fbxStr);
}

} // namespace
