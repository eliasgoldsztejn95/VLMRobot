## Folder descriptions

1) external: Includes the files to run in the server computer. Currently it is impleneted in an external computer that is monted in the robot, however it can be all written in the robot computer.
2) robot: It includes the file that connects with the external computer. It assumes the robot runs ROS1 and has a T256 odometry sensor and RPlidar. It is a minimal code and can be costumized for ROS2 or differnt sensor setup.
3) user: It includes the files to send messages and receive videos to the robot for remote control. It assumes you are in the same wifi network.
4) notification: It includes the files needed for the notification app. Ktor needs to be running in the external computer. The app is implemented for Android.
