# Temi ROS driver via MQTT

This repo implement a ROS interface to interact with robotemi's Android SDK.

## Features (Work in Progress)
* Subscriber
  * call any API provided by sdk via 'cmd' topic.
  * cmd_vel topic for controlling temi (using skidJoy)
    * currently running at 10hz

* Publisher
  * temi 2d pose topic (x, y, yaw, tiltAngle)
  * camera_image topic

## Dependencies
* Docker (for running MQTT broker)

