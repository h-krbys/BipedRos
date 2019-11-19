# choreonoid_biped

### 20191027 kuribayashi:

ROS上でChoreonoidを利用するための手順を確認

	$ mkdir cnoid_ws
	$ cd cnoid_ws
	$ mkdir src
	$ catkin init
	$ cd src
	$ git clone https://github.com/s-nakaoka/choreonoid.git
	$ git clone https://github.com/s-nakaoka/choreonoid_rosplugin.git
	$ git clone https://github.com/s-nakaoka/choreonoid_ros_samples.git
	$ git clone https://github.com/s-nakaoka/choreonoid_joy.git
  $ git clone git@133.30.142.15:~/choreonoid_biped
	$ cd choreonoid
	$ git clone git@133.30.142.15:~/choreonoid/BipedCnoid
	$ cd ../..
	$ catkin build

BipedCnoidをビルドする

	$ catkin config --cmake-args -DADDITIONAL_EXT_DIRECTORIES=~/catkin_ws/src/choreonoid/BipedCnoid -DUSE_PYTHON3=OFF -DBUILD_BIPEDCNOID_PLUGIN=ON -DBUILD_BIPEDCNOID_SAMPLE=ON