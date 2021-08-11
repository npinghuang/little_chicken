SOP to run little chicken:

!!set master on your own computer!!


1. launch lidar and communication on pi:
roslaunch testbot serial.launch

2. launch lidar localization on your own computer:
roslaunch lidar_localization lidar_localization.launch

3. lauch main:
roslaunch main2021 small_main0324_demo.launch

on gui: choose strategy 4, click prepare, set

and to start type:
rosparam set /start 1 
