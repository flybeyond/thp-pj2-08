# Introduction #

Use the instruction on this page to install the packages necessary for controlling the delta robot.

# Details #

Edit your environment variables. Add this to your ~/.bashrc:

```
export ROS_PACKAGE_PATH=~/ros_packages:$ROS_PACKAGE_PATH
```

Do a checkout
```
$ svn co http://thp-pj2-08.googlecode.com/svn/trunk/ thp-pj2-08-read-only
```

Compile the packages with _make_ in the packages source directory.

Start ros:

```
$ roscore
```

Start the packages:

```
$ rosrun par_trajectory_planning par_planning
$ rosrun par_kinematics par_kinematics 
$ rosrun par_ui par_ui
```

The package par\_ui is the package that allows a user to control the delta robot.