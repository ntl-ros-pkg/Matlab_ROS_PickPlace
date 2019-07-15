cmake_minimum_required(VERSION 2.8.3)
project(motoman_description)
find_package(catkin REQUIRED)
catkin_package()

###########
# Install #
###########
set(LAUNCH_DIRS
  launch/motomini
  launch/motomini/rviz
  launch/sda5
  launch/sda5/rviz
  launch/sia5
  launch/sia5/rviz
  launch/sia20
  launch/sia20/rviz
)
set(MESH_DIRS
  meshes/motomini/collision
  meshes/motomini/visual
  meshes/sda5/collision
  meshes/sda5/visual/base_link
  meshes/sda5/visual/link_b
  meshes/sda5/visual/link_e
  meshes/sda5/visual/link_l
  meshes/sda5/visual/link_r
  meshes/sda5/visual/link_s
  meshes/sda5/visual/link_t
  meshes/sda5/visual/link_u
  meshes/sda5/visual/torso_link
  meshes/sia5/collision/STL
  meshes/sia5/visual/COLLADA
  meshes/sensor/visual/COLLADA
  meshes/sia20/base_link
  meshes/sia20/link_s
  meshes/sia20/link_l
  meshes/sia20/link_e
  meshes/sia20/link_u
  meshes/sia20/link_r
  meshes/sia20/link_b
  meshes/sia20/link_t
  meshes/tool
)
set(URDF_DIRS
  robots/motomini
  robots/sda5
  robots/sia5
  robots/sia20
  urdf
  urdf/motomini
  urdf/sda5
  urdf/sia5
  urdf/sia20/base_link
  urdf/sia20/link
  urdf/sensor
  urdf/tool
)

foreach(dir ${LAUNCH_DIRS} ${MESH_DIRS} ${URDF_DIRS})
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

########
# Test #
########
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/motomini)
  roslaunch_add_file_check(launch/sda5)
  roslaunch_add_file_check(launch/sia5)
  roslaunch_add_file_check(launch/sia20)
endif()

