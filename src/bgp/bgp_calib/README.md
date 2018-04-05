# BGP: Bernd's Groundtruth Package

This is meant to become a larger package to determine ground truth poses for visual-inertial odometry experiments.
For now it does a very simple multi-camera extrinsic calibration.

## How to install

Install GTSAM from the ppa:

    sudo apt-add-repository ppa:bernd-pfrommer/gtsam
	sudo apt update
	sudo apt install gtsam

Get the necessary packages in place

    cd ~/catkin_ws/src
	git clone https://github.com/berndpfrommer/bgp.git
	# install apriltag_ros package 
	git clone https://github.com/versatran01/apriltag.git
	# catkin_simple
	git clone https://github.com/catkin/catkin_simple.git

Now build:

    catkin_cmake

or:

	catkin build

## how to run the tests

Here is how to run the extrinsic calibration tests:

    roslaunch bgp_calib test.launch 

