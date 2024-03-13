# Additional Hardware Description
This document provides some addition description of the harware for the spot micro robot.

* [Custom 3d Printed Parts](#custom-3d-printed-parts)
* [Coordinate Frames](#coordinate-frames)
* [Sample Hardware Install Photos](#sample-hardware-installation-photos)


## Custom 3d Printed Parts

KDY의 원래 디자인에서 확장하기 위해 여러 개의 맞춤형 3D 인쇄물이 만들어졌습니다. 여기에는 추가 보강재를 제공하기 위한 맞춤형 어깨 조립품과 편의를 위한 여러 장착 플랫폼이 포함됩니다.

아래에 표시된 수정된 어깨 조립품은 어깨 축에 추가 보강을 제공하기 위해 추가 플라스틱 조각을 포함합니다. [thingverse page](https://www.thingiverse.com/thing:4591999) 내용을 참조하여 어깨 관절 부품의 전체 세트를 인쇄해야 합니다. 반대쪽 다리에 대해서는 두 세트를 좌우반전하여 인쇄해야 합니다. 수정된 어깨에는 조립을 위해 추가로 8x M3x10 나사, 8x M3 너트 및 4x F625z 베어링이 필요합니다.


![Custom Shoulder Assembly](../assets/custom_shoulder_assembly.jpg)

일반 중앙 본체 플랫폼과 RPI 3 및 PCA9685 보드를 장착할 수 있는 두 개의 편의 플랫폼은 [thingverse page](https://www.thingiverse.com/thing:4596267)에서 찾을 수 있습니다. Raspberry Pi 플랫폼은 아래에 나와 있습니다. 양면 폼 테이프로 중앙 플랫폼에 부착할 수 있습니다. RPi3 및 PCA9685를 이 플랫폼에 부착하려면 작은 나무 나사를 사용할 수 있습니다.

![RPI platform](../assets/rpi_platform.jpg)

RPLidar A1을 위한 맞춤형 플랫폼 및 마운트 어댑터는 [at this thingverse page](https://www.thingiverse.com/thing:4713180)에서 확인할 수 있습니다. 디자인은 Maggie Mathiue가 제공합니다. 위 플랫폼과 마찬가지로 양면 폼 테이프로 베이스를 중앙 몸체 플랫폼에 부착할 수 있습니다. 마운트 어댑터는 RPLiadr A1에 4x M2.5x8 나사로 부착하고 바닥 플랫폼에는 작은 나무 나사로 부착합니다.

![lidar mount](../assets/lidar_mount.jpg)



## Coordinate Frames
There are many coordinate frames on the spot micro frame (one at each joint!), but some of the more important frames are described here.


#### Kinematics Coordinate Frame
With regard to kinematics of the robot frame, the coordinate frame is oriented as follows: X positive forward, Y positive up, Z positive left. This frame was is only relevant if working on the kinematic calculations for the robot. It is the same coordinate frame as used in the paper sourced for the inverse kinematic calculations for this project ("Inverse Kinematic Analysis Of A Quadruped Robot").

![Kinematic coordinate system](../assets/kinematic_coord_system.jpg)

#### TF2 Coordinate Frame 
The TF2 coordinate frames is the base robot body coordinate frame used for all transforms published to TF2 within the ROS framework. This is the robot coordinate frame of interest with regard to mapping and navigation. This frame is oriented as follows: X positive forward, Y positive left, Z positive up.

![tf2 coordinate system](../assets/tf2_coord_system.jpg)


## Sample Hardware Install Photos
The following photos a sample installation of components on the spot micro frame.


![Robot top no servos](../assets/robot_top_no_servos.jpg)

![tf2 coordinate system](../assets/robot_top.jpg)

![tf2 coordinate system](../assets/robot_bottom.jpg)
