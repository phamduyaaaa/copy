# copy
```bash
<?xml version="1.0"?>
<robot name="my_mecanum_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="chassis_length" value="0.35"/>
  <xacro:property name="chassis_width" value="0.25"/>
  <xacro:property name="chassis_height" value="0.1"/>
  
  <xacro:property name="lidar_offset_x" value="-0.025"/> 
  <xacro:property name="lidar_offset_z" value="0.08"/> 
  <xacro:property name="imu_offset_z" value="0.05"/>

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
    <visual>
        <geometry>
            <box size="0.05 0.05 0.05"/>
        </geometry>
        <origin xyz="${chassis_length/2 - 0.025} 0 ${chassis_height/2 + 0.025}" rpy="0 0 0"/>
        <material name="red"/>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/> 
    <origin xyz="${lidar_offset_x} 0 ${lidar_offset_z}" rpy="0 0 0"/>
  </joint>

  <link name="laser_frame">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="red"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${imu_offset_z}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

</robot>
```
