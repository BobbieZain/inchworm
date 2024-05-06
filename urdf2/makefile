pkg1_name := movement
pkg2_name := final_description
launch_file := gazebo.launch.py

.PHONY: install
.PHONY: run
install:
	rm -r build log install
	colcon build
	
gazebo:
	colcon build&&. install/local_setup.sh && ros2 launch ${pkg1_name} ${launch_file}
