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
스팟 마이크로 프레임에는 많은 좌표 프레임이 있지만(각 조인트에 하나씩!), 더 중요한 프레임 중 일부는 여기에 설명되어 있습니다.

#### Kinematics Coordinate Frame
로봇 프레임의 운동학과 관련하여 좌표 프레임은 X 양의 앞, Y 양의 위, Z 양의 왼쪽으로 향합니다. 이 프레임은 로봇의 운동학 계산을 수행하는 경우에만 관련이 있습니다. 이 프로젝트의 역 운동학 계산을 위해 논문에서 제공한 것과 동일한 좌표 프레임입니다("네 발 달린 로봇의 역 운동학 분석").

![Kinematic coordinate system](../assets/kinematic_coord_system.jpg)

#### TF2 Coordinate Frame 
TF2 좌표 프레임은 ROS 프레임워크 내에서 TF2에 게시되는 모든 변환에 사용되는 기본 로봇 본체 좌표 프레임입니다. 매핑 및 탐색과 관련하여 관심 있는 로봇 좌표 프레임입니다. 이 프레임은 X 양의 전방, Y 양의 좌측, Z 양의 상방으로 향합니다.

![tf2 coordinate system](../assets/tf2_coord_system.jpg)


## Sample Hardware Install Photos
다음은 스팟 마이크로 프레임에 구성 요소를 설치한 샘플 사진입니다.


![Robot top no servos](../assets/robot_top_no_servos.jpg)

![tf2 coordinate system](../assets/robot_top.jpg)

![tf2 coordinate system](../assets/robot_bottom.jpg)
