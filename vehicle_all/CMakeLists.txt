cmake_minimum_required(VERSION 2.8.3)

project(vehicle_all)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Install scripts
install(PROGRAMS
  scripts/vehicle_follower.py
  scripts/test_vehicle_control.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)