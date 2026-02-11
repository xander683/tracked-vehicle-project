cmake_minimum_required(VERSION 2.8.3)

project(vehicle_all)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf maps worlds rviz)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Install scripts
install(PROGRAMS
  scripts/vehicle_follower.py
  scripts/vehicle_teleop.py
  scripts/diff_drive_odom.py
  scripts/control_vehicle1.py
  scripts/control_vehicle_gazebo.py
  scripts/record_vehicles_data.py
  scripts/record_vehicles_gazebo.py
  scripts/visualize_vehicles_data.py
  scripts/analyze_vehicles_data.py
  scripts/s_trajectory_driver.py
  scripts/s_curve_leader_pp.py
  scripts/obstacle_spawner.py
  scripts/s_curve_pp_pid_driver.py
  scripts/plot_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
