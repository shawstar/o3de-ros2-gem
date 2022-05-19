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
    std::string GetUrdfWithOneLink()
    {
        std::string xmlStr =
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
        return xmlStr;
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

    std::string GetSimpleRobotUrdf()
    {
        std::string xmlStr =
            "<robot name=\"simple_robot\">"
            "  <material name=\"black\">"
            "    <color rgba=\"0.0 0.0 0.0 1.0\"/>"
            "  </material>"
            "  <material name=\"blue\">"
            "    <color rgba=\"0.0 0.0 0.8 1.0\"/>"
            "  </material>"
            "  <link name=\"base_link\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"blue\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"wheel_bl\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"wheel_br\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"wheel_fl\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"wheel_fr\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"head\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            "  <link name=\"camera_link\">"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "      <material name=\"black\"/>"
            "    </visual>"
            "  </link>"
            // Joints
            "  <joint name=\"base_link_to_wheel_bl\" type=\"continuous\">"
            "    <parent link=\"base_link\"/>"
            "    <child link=\"wheel_bl\"/>"
            "    <origin xyz=\"-0.5 -0.25 0.0\" rpy=\"0 0 0\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "  </joint>"
            "  <joint name=\"base_link_to_wheel_br\" type=\"continuous\">"
            "    <parent link=\"base_link\"/>"
            "    <child link=\"wheel_br\"/>"
            "    <origin xyz=\"-0.5 0.25 0.0\" rpy=\"0 0 0\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "  </joint>"
            "  <joint name=\"base_link_to_wheel_fl\" type=\"continuous\">"
            "    <parent link=\"base_link\"/>"
            "    <child link=\"wheel_fl\"/>"
            "    <origin xyz=\"0.5 -0.25 0.0\" rpy=\"0 0 0\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "  </joint>"
            "  <joint name=\"base_link_to_wheel_fr\" type=\"continuous\">"
            "    <parent link=\"base_link\"/>"
            "    <child link=\"wheel_fr\"/>"
            "    <origin xyz=\"0.5 0.25 0.0\" rpy=\"0 0 0\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "  </joint>"
            "  <joint name=\"base_link_to_head\" type=\"continuous\">"
            "    <parent link=\"base_link\"/>"
            "    <child link=\"head\"/>"
            "    <origin xyz=\"0.0 0.0 0.5\" rpy=\"0 0 0\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "  </joint>"
            "  <joint name=\"head_to_camera_link\" type=\"continuous\">"
            "    <parent link=\"head\"/>"
            "    <child link=\"camera_link\"/>"
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

    // PrintFbxContent(fbxStr);
}

TEST_F(UrdfToFbxConverterTest, ConvertSimpleRobotUrdf)
{
    const auto urdfStr = GetSimpleRobotUrdf();
    const auto fbxStr = converter.Convert(urdfStr);

    // PrintFbxContent(fbxStr);
}

} // namespace
