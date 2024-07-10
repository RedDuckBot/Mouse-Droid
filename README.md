# Mouse_Droid
A differential drive mouse droid, from the Star Wars universe, that was design for entertaining people at a house party. The droid is equiped with a revolving disco ball and can make actual mouse droid sounds heard from the movies. It is manually operated using an Xbox 360 wired controller. Overall, the operations of droid involves a raspberry pi 4B+, utilizing Ubuntu Server 22.04, and the running software application is built on the middleware ROS2 Humble. Majority of the parts are 3D printed and the droid consists of many other basic electronic components. Using a PID controller the droid is able to do a 180 degree or complete rotation.    

<p align='center'>
    <img src=docs/images/mouse_droid_clip.gif width="600">
</p>
<p align='center'>
    <img src=docs/images/droid.png width="600">
</p>


## 🗃️ Package Overview
- [`droid_server_commands`](./droid_setup/ros2_ws/src/droid_server_commands/) : Contains action server nodes for handling controller input
- [`droid_controller`](./remote_setup/ros2_ws/src/droid_controller/) : Contains action client node that casts the inputs for the Xbox 360 controller
- [`droid_interfaces`](./remote_setup/ros2_ws/src/droid_interfaces/) : Contains the ROS2 actions and messages used in this project representing the data for Xbox 360 controller


## 🧰 Hardware

<p align='center'>
    <img src=docs/images/droid_compartment.png width="600">
</p>


| | Electronic Components |
| --| --|
|1| Raspberry Pi 3B+ |
|2| Lexar 128 GB SD Card |
|3| Sound Board DY-HV20T |
|4| 4 Ohm 5W Audio Speaker |
|5| Motor Controller L298N  |
|6| 4 x DC-12V-200RPM Motors|
|7| DC 5V Stepper Motor 28BYJ-48 |
|8| ULN2003 Step Driver Board |
|9| [12mm (Outside Diameter) Slip Ring](https://www.amazon.ca/dp/B07H2SRMXP?psc=1&ref=ppx_yo2ov_dt_b_product_details) |
|10| [Mini USB Disco Ball with 62mm Diameter](https://www.amazon.ca/dp/B0BK6Z3LYH?ref=ppx_yo2ov_dt_b_product_details&th=1) |
|11| 12V, 3800mAh LIPO Battery |
|12| 5V, 26000mAh Battery Bank |

### 3D Printed Parts

The parts for the droids body were taken from a maker on Thingiverse, [Null_Hypothesis](https://www.thingiverse.com/null_hypothesis/designs). Here's the link to his project https://www.thingiverse.com/thing:4229599.
From his project only the following parts are needed, given that the design needed to be hacked to work for a diff drive droid:
- 2 x Chassis_Rear_V4.stl
- Top_FrontV4.stl
- Top_RearV4.stl
- Top_rightV4.stl
- Top_RoofV4.stl
- TopLeftV4.stl
- 2 x Mouse_Droid_Side_Circuit_Board_Rescaled.stl   
- 2 x MSE_base 6to10.stl
- 2 x MSE_Lower_Portion6to10.stl
- 2 x MSE_Upper_Portion 6to10.stl
- 4 x Front_Tire.stl

The parts mentioned above and custom parts made for this project are in the 3D_print_files directory.
Portions of the chassis where the wheels go were cut out for placing the custom motor mounts, and a 64mm hole was cut out from the droid's roof for dico ball placement. Moreover, the custom disco platforms, made to hold electronic and mechanical components for operating the it, were suspended bellow the droid's roof using hex brass standoffs. [`Images`](./docs/images/) were taken to show assembeled components.  

## Circuit Diagrams 
### Disco Ball Circuit
<p align="center">
  <img title='Disco Ball Circuit' src=docs/images/discoBall_circuit.png width="800">
</p>
Rotational control for the disco ball was done using a custom circuit, as shown above, involving a 555 timer, 4017 Decade counter, few resistors, capacitor, and an N-mosfet. This circuit generates steps for the stepper motor and was controlled off a pin from the Raspberry Pi that connected to the mosfet; operation can easily be achieved by directly connecting the Pi's pins to the ULN2003 Step board, instead of making and using the custom circuit.
Note: since the disco ball uses a USB connection a portion of the metal connector was cut out to expose the Vin and GND pins, which were soldered to the spin wires of the slip ring with the out wires connecting to the mosfet.


### Sound Board, Motor Controller, and IMU Circuit
<p align="center">
  <img title='Sound Board, Motor Controller, and IMU Circuit' src=docs/images/soundboard_motorController_IMU_circuit.png width="800">
</p>
In the above diagram, two power sources are utilized: 3s LIPO battery for powering the sound board and motors, and 5V battery bank for the Raspberry PI and disco circuit.

## Setup for operating the Droid 

Make sure ROS2 Humble is installed on the Raspberry PI, while the remote computer for controlling the droid does not need to install Humble since in this setup a Docker container is utilized for easy setup.
Copy the directory [`remote_setup`](./remote_setup/) onto the remote machine, and [`droid_setup`](./droid_setup/) onto the Raspberry PI.

### Remote Setup
For network communication, this project utilizes Eclipse Cyclone DDS, so before building and running the Docker container, add both the PI's and remote computer's IP addresses to the [`cyclon file`](./remote_setup/cyclonedds.xml). Moreover, this document provides comments for where to place these addresses. 

After modifying the cyclon file, build & run the container:

```
Scripts/build.bash
```
Next, run the container but make sure the Xbox 360 controller is connected.

```
Scripts/run.bash
```

Finally, before running the controller node, the ROS2 packages for this setup need to be compiled and installed.

```
cd ws/
colcon build
source install/setup.bash
ros2 run droid_controller controller
```
Expect similar output, after running the controller node: [INFO] [1720638165.505265926] [controller]: Xbox controller node initialized.


### Droid Setup
Make sure the library [pigpio](https://abyz.me.uk/rpi/pigpio/) is installed on the Raspberry PI.
Given permission issues encountered when trying to run the droid server node on the PI, the work around solution is running this node while logged in as the root user. 

Before compiling and installing the ROS2 packages for this setup, run the following commands:
```
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI= ./droid_setup/cyclonedds.xml #Enter the full location of the cyclon DDS file
```

Then, as done in the remote setup, add the PI's and remote computer's IP addresses to the cyclon file.
Next, while in the the directory ./droid_setup/ros2_ws, compile the droids ROS2 packages:

```
colcon build
source install/setup.bash
```

Finally, run the droid server node which takes commands from the controller node:
```
ros2 run droid_server_commands droid_server
```

Expect similar output, after running this node: 
```
[INFO] [1720639837.026257729] [Droid_Server]: IMU Module Ready
[INFO] [1720639837.164474318] [Droid_Server]: Sound Module Ready
[INFO] [1720639837.164727443] [Droid_Server]: Droid servers started
```

### Xbox 360 Controller Commands
- Left Joy-stick ---> operates left side motors
- Right Joy-stick ---> operates right side motors
- Back button ---> Droid performs an automatic 180 deg rotation
- Mode button (between back & start buttons) ---> Droid performs an automatic 360 deg rotation
- X, Y, B, A buttons ---> Perform droid sounds
- Start button ---> toggles disco ball on and off

The droid sounds are in resources and can be installed to an SD for the sound board. 


## Acknowledgements
- [IMU Module](https://github.com/CVino/RPi_BNO055)
- [Motor Wrapper](https://github.com/buildrobotsbetter/rpi4b_gpio-example)
- [Sound Module](https://github.com/SnijderC/dyplayer)