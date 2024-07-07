# Mouse_Droid
A differential drive mouse droid, from the Star Wars universe, that was design for entertaining people at a house party. The droid is equiped with a revolving disco ball and can make actual mouse droid sounds heard from the movies. It is manually operated using an Xbox 360 wired controller. Overall, the operations of droid involves a raspberry pi 4B+, utilizing Ubuntu Server 22.04, and the running software application is built on the middleware ROS2 Humble. Majority of the parts are 3D printed and the droid consists of many other basic electronic components.    

<p align='center'>
    <img src=docs/images/mouse_droid_clip.gif width="600">
</p>

## 🗃️ Package Overview
- [`droid_server_commands`](./droid_setup/ros2_ws/src/droid_server_commands/) : Contains action server nodes for handling controller input
- [`droid_controller`](./remote_setup/ros2_ws/src/droid_controller/) : Contains action client node that casts the inputs for the Xbox 360 controller
- [`droid_interfaces`](./remote_setup/ros2_ws/src/droid_interfaces/) : Contains the ROS2 actions and messages used in this project representing the data for Xbox 360 controller

