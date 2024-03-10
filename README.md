# Spot Micro Quadruped Project (번역본)

![Spot Micro Walking](assets/spot_micro_walking.gif)
![RVIZ](assets/rviz_animation.gif)
![slam](assets/spot_micro_slam.gif)


Video of robot: https://www.youtube.com/watch?v=S-uzWG9Z-5E

* [Overview](#Overview)
* [General Instructions](#general-instructions)
* [Description of ROS Nodes](#description-of-ros-nodes)
* [Additional Project Components](#additional-project-components)
    * [URDF](#urdf-model)
    * [TF2 Publishing and Odometry](#tf2-publishing-and-odometry)
    * [SLAM](#slam)
* [Future Work](#future-work)
* [External Links](#external-links)

## Overview
본 프로젝트는 4족 오픈소스 로봇인 Spot Micro Quadruped의 소스코드입니다. 이 코드는 앉기, 서기, 각도 및 걷기 제어를 포함하여 3D 프린팅된 스팟 마이크로 로봇의 모션 제어를 구현합니다. 지원 라이브러리는 SLAM을 통한 매핑 및 본체 장착 라이더와 같은 추가 기능을 제공합니다. 이 소프트웨어는 ROS Kinetic이 설치된 Ubuntu 16.04를 실행하는 Raspberry Pi 3B에서 구현됩니다.

이 소프트웨어는 ROS 프레임워크의 C++ 및 Python 노드로 구성됩니다.

#### Hardware:
활용된 프레임은 KDY0523에서 개발한 Thingverse Spot Micro 프레임입니다. 조립 하드웨어에 대한 자세한 내용은 [thingverse 페이지](https://www.thingiverse.com/thing:3445283)를 참조하세요. 이 프로젝트에서 사용한 hv5523mg 서보에도 맞는 cls6336hv 서보용 파일이 인쇄되었습니다.

Component List:
* 마이크로컨트롤러: Raspberry Pi 3B 
* 서보 모터 드라이버: PCA9685 (i2c 방식으로 제어)
* 서보 모터: PDI-HV5523MG x 12개
* LCD 패널: 16x2 i2c LCD panel (선택사항항)
* 배터리: 2s 4000 mAh 리튬 폴리머 배터리 (서보 모터 드라이버에 직접 연결)
* UBEC(전원공급장치): HKU5 5V/5A ubec (라즈베리 파이, lcd 패널, pca9685에 5V 전원 공급)
* Lidar: RPLidar A1
* 전용 3D 프린트 마운트

추가 맞춤형 3D 프린팅 부품, 좌표계 정보, 샘플 하드웨어 설치 사진 등 하드웨어에 대한 자세한 내용은 [추가 하드웨어 설명](docs/additional_hardware_description.md) 문서에서 확인할 수 있습니다.


#### Software:
이 저장소는 Ubuntu 16.04의 ROS Kinetic 환경에서 catkin 작업 공간으로 구성되어 있습니다. 소프트웨어는 이 환경 외부에서 작동하거나 컴파일되지 않을 수 있습니다. Ubuntu 16.04 및 ROS Kinetic 설치가 사전 로드된 Raspberry Pi 이미지는 유비쿼터스 로봇공학 웹페이지를 통해 찾을 수 있습니다. 다운로드, 설정, Wi-Fi 설정 지침은 [유비쿼터스 로봇공학 웹페이지](https://downloads.ubiquityrobotics.com/)를 참조하세요. 개발 및 제어 노드 실행을 위해 PC의 Ubuntu 16.04 Linux 설치/듀얼 부팅/가상 머신에 ROS Kinetic을 설치하는 것도 권장됩니다. ROS kinine 설치 지침은 [여기](http://wiki.ros.org/dynamic/Installation/Ubuntu)에서 확인할 수 있습니다.

**참고** RPI의 온보드 RAM 이상으로 사용 가능한 가상 메모리를 늘리려면 RPI의 SD 카드에 약 1GB의 SWAP 파티션이 필요합니다. 경험적으로 SWAP 파티션을 추가하지 않으면 catkin 컴파일 프로세스는 온보드 RAM을 모두 사용하고 무한정 정지되며 완료되지 않습니다. SWAP 파티션 추가에 대한 예제 지침은 [여기에서 찾을 수 있습니다](https://nebl.io/neblio-university/enabling-increasing-raspberry-pi-swap/).

제공된 ROS Catkin make 빌드 시스템을 활용할 수 있지만 대신 'catkin tools'를 사용했습니다([catkin tools 웹사이트 참조]((https://catkin-tools.readthedocs.io/en/latest/))). 아래 컴파일 명령은 `catkin tools`를 가정하여 제공됩니다. 라즈베리 파이에서 catkin 도구를 사용하지 않는 경우 스톡 `catkin_make`를 사용하여 catkin 작업 공간 홈에서 `catkin_make -DCMAKE_BUILD_TYPE=Release`와 같은 명령을 통해 코드를 컴파일할 수 있습니다.

##### Software Checkout and Setup:
이 저장소는 라즈베리 파이의 catkin 작업 공간으로 체크아웃되어야 디렉토리 구조가 아래와 같이 나타납니다. 아직 사용할 수 없는 경우 catkin 도구를 사용하여 catkin 작업 공간을 만들거나 catkin make 작업 공간에서 전환할 수 있습니다([또는 기본 ROS 도구를 사용하는 경우 catkin 작업 공간 만들기에 대한 튜토리얼 페이지 참조](http://wiki.ros.org/) catkin/Tutorials/create_a_workspace)). pi가 인터넷에 연결되어 있지 않으면 catkin 명령을 사용하여 다른 컴퓨터에 작업 공간을 만든 다음 scp를 통해 wifi를 통해 RPi에 파일을 복사할 수 있습니다. 예: `scp spotMicro/* ubuntu@10.42.0.1:~/catkin_ws/src/`.

```
catkin_ws/
│
├── src/
│   ├── spot_micro_motion_cmd
│   │   └── ...
│   ├── spot_micro_keyboard_cmd
│   │   └── ...  
│   └── ...
```

이 저장소는 두 개의 git 하위 모듈을 사용하며 이를 확인하려면 추가 단계가 필요합니다. 기본 저장소를 확인한 후 다음을 통해 하위 모듈을 확인하세요.

```
git submodule update --init --recursive
git submodule update --recursive
```

git 권한 오류가 발생하면 [이 stackoverflow 게시물](https://stackoverflow.com/questions/8197089/fatal-error-when-updating-submodule-using-git)을 통해 다음 제안을 시도해 보세요.

이 프로젝트를 성공적으로 빌드하려면 세 개의 추가 ROS 패키지를 설치해야 할 수도 있습니다. 다음을 통해 설치할 수 있습니다.

```
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-rplidar-ros
sudo apt-get install ros-kinetic-hector-slam
```

동일한 저장소가 pi와 노트북/PC 모두에서 체크아웃되므로 소프트웨어가 올바르게 컴파일되도록 노트북/PC에 i2c 라이브러리를 설치해야 합니다. `i2cpwm_board` 노드는 노트북/PC에서 실행되지 않지만 컴파일은 이 노드에 대한 종속성을 찾습니다. 다음을 통해 필요한 라이브러리를 설치하십시오:
`sudo apt-get install libi2c-dev`

cmake 릴리스 플래그가 추가되도록 catkin 도구를 구성합니다. 이렇게 하면 코드 실행 속도가 빨라집니다. 또는 VSCode와 같은 IDE를 통해 디버깅하려면 빌드 유형 디버그를 사용하여 디버그 기호가 생성되도록 하세요:
`catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`

catkin 도구를 통해 spot_micro_motion_cmd 및 i2cpwm_board 노드를 컴파일합니다. 아래 명령은 종속성이므로 프로세스에서 i2cpwmboard를 자동으로 빌드합니다:
`catkin build spot_micro_motion_cmd` 

아니면 전체 프로젝트를 빌드하세요:
`catkin build`

pi에서 실행할 때 아래와 같은 오류가 발생하면 다운로드한 rpi 이미지에 libi2c-dev가 설치되지 않았을 가능성이 높습니다. 이 문제를 해결하려면 `apt-get` 명령을 사용하여 pi에 라이브러리를 설치할 수 있습니다. pi에 인터넷이 없으면 우분투 16.04에 적합한 버전이 있는 기본 컴퓨터에 데비안 `.deb` 패키지로 파일을 다운로드할 수 있습니다(https://ubuntu.pkgs.org/16.04/ubuntu-universe-amd64/libi2c-dev_3.1.1-1_all.deb.html) 그런 다음 `scp`를 통해 파일을 pi에 복사합니다(`scp libi2c-dev_3.1.1-1_all.deb ubuntu@10.42.0.1:~/`) 그리고 수동으로 설치하세요(`sudo dpkg -i libi2c-dev_3.1.1-1_all.deb`).
```
ros-i2cpwmboard/CMakeFiles/i2cpwm_board.dir/build.make:62: recipe for target 'ros-i2cpwmboard/CMakeFiles/i2cpwm_board.dir/src/i2cpwm_controller.cpp.o' failed
make[2]: *** [ros-i2cpwmboard/CMakeFiles/i2cpwm_board.dir/src/i2cpwm_controller.cpp.o] Error 1
CMakeFiles/Makefile2:2343: recipe for target 'ros-i2cpwmboard/CMakeFiles/i2cpwm_board.dir/all' failed
```

#### Note on Walking Gaits
구현된 기본 보행은 균형과 안정성을 유지하는 데 도움이 되는 신체 움직임을 통합한 8 phase gait입니다. 대각선으로 반대쪽 다리가 동시에 움직이는 대체 trot gait은 더 빠른 보행 속도를 달성할 수 있지만 안정성이 낮고 로봇 질량 중심의 신중한 위치 지정이 필요합니다. trot gait은 이 문서 상단의 애니메이션에 묘사된 것입니다. trot gait으로 전환하는 방법에 대한 자세한 내용은 `spot_micro_motion_cmd` 노드의 구성 파일을 참조하세요. 8 phase gait은 링크된 유튜브 영상에서 보실 수 있습니다.

## General Instructions
이 섹션에서는 이 코드를 사용하여 스팟 마이크로 로봇을 보정하고 실행하기 위한 전체 지침 세트를 시도합니다.

#### Servo Configuration

서보 설치, 교정 및 구성에 대한 포괄적인 지침은 [servo_calibration](docs/servo_calibration.md) 문서에서 찾을 수 있습니다.

#### Running:
적어도 하나의 SSH를 통해 라즈베리 파이에 연결된 두 개 이상의 터미널 창을 엽니다. 편의를 위해 `tmux`와 같은 터미널 멀티플렉서를 사용하는 것이 좋습니다. 해당 터미널에서 다음 실행 파일을 시작합니다:
* `roslaunch spot_micro_motion_cmd motion_cmd.launch`: 라즈베리파이에서 실행해보세요. i2c_pwmboard 노드와 로봇의 모션 제어 노드를 시작합니다. 시작 시 모션 명령 노드는 i2c_pwmboard 노드에 서보 구성 메시지를 보낸 다음 상태 머신을 시작하고 유휴 상태에 들어갑니다.
* `roslaunch spot_micro_keyboard_command keyboard_command.launch`로컬 컴퓨터에서 실행합니다. 스팟 마이크로 로봇에 키보드 명령을 내리기 위한 키보드 명령 노드를 시작합니다.
* **선택사항**: 위의 두 시작 파일은 추가 노드를 시작하기 위해 선택적 명령줄 인수를 사용할 수 있습니다. 명령줄 인수 중 일부가 아래에 나열되어 있습니다.
    * Command line arguments for `motion_cmd.launch`:
        * `run_standalone:=true`: Runs the motion command node standalone (without running the i2c_pwmboard node)
        * `debug_mode:=true`: Overrides the `debug_mode` parameter to true. Useful in combination with `run_standalone` for running or debugging the motion command node on a PC instead of the RPi
        * `run_lcd:=true`: Runs the lcd monitor node to display simple state information on a LCD monitor if installed. Only works running on a RPi
    * Command line arguments for `keyboard_command.launch`:
        * `run_rviz:=true`: Starts RVIZ and displays a 3d model of the robot with state update in real time.
        * `run_plot:=true`: Runs python plotting node to display a stick figure wireframe model of the spot micro robot state in real time. Must be run on a PC. Requires updated matplot lib python library (matplotlib 2.2.5) and updated numpy library (numpy 1.16.6).
* For running SLAM, see the [SLAM document](docs/slam_information.md) for more information, which is also referenced in the [Additional Project Components](#additional-project-components) section.
        

키보드 명령 실행 파일을 중지하고 종료하려면 'quit'을 입력하고 'Ctrl-C'를 눌러야 할 뿐만 아니라 플로팅이 활용된 경우 모든 플롯 창을 닫아야 할 수도 있습니다.

명령줄 인수는 `roslaunch` 명령 끝에 추가되며 여러 인수를 한 번에 추가할 수 있습니다. 예를 들면 다음과 같습니다:
`roslaunch spot_micro_motion_cmd motion_cmd.launch run_standalone:=true debug_mode:=true`

실행 파일은 포함되어 있어 편리하지만 필요한 경우 'rosrun'을 통해 노드를 개별적으로 실행할 수 있습니다(매개변수를 읽기 위해 실행 파일에서 시작해야 하는 spot_micro_motion 명령 제외). 예를 들어, 세 개의 별도 터미널에서 다음 세 가지 명령을 사용하여 최소 필수 노드를 실행할 수 있습니다:
* `rosrun i2cpwm_board i2cpwm_board` 
* `roslaunch spot_micro_motion_cmd motion_cmd.launch run_standalone:=true`
* `rosrun spot_micro_keyboard_command spotMicroKeyboardMove.py`


#### Control instructions:
로봇 소프트웨어는 신중한 제어 모드의 유한 상태 기계에 의해 구동됩니다. spot_micro_keyboard_command 노드를 실행하는 터미널에서는 상태 머신을 통해 이동하고 본체 속도 및 각도 명령을 명령하기 위해 키보드 명령이 실행됩니다.

모든 소프트웨어가 시작된 후 키보드 제어 노드 터미널에 `stand` 명령을 입력하고 실행하여 로봇이 일어서도록 명령합니다. 여기에서 `idle` 명령을 실행하여 다시 앉아서 서보를 유휴 상태로 설정하거나 `angle_cmd`를 실행하여 신체 방향 각도를 명령하거나 `walk`를 실행하여 걷기 모드로 들어갈 수 있습니다.

angle_cmd 모드에서 `w`와 `s` 키는 피치를 제어하는 ​​데 사용되며 `a`와 `d`는 롤을 제어하고 `q`와 `e`는 요를 제어하는 ​​데 사용됩니다. `u`는 제어 모드 입력으로 돌아가는 데 사용됩니다.

walk 모드에서 `w` 및 `s` 키는 전진 속도를 제어하는 ​​데 사용되며 `a` 및 `d`는 측면 속도를 제어하고 `q` 및 `e`는 요 속도를 제어하는 ​​데 사용됩니다. `u`는 스탠드 모드로 돌아가는 데 사용됩니다.

소프트웨어는 현재 명령 제한을 지원하지 않으므로 로봇의 능력을 넘어서는 방향이나 신체 속도를 명령하는 경우 해롭거나 하드웨어에 손상을 주는 동작이 발생할 수 있습니다.

명령 입력에 `quit`을 입력하면 이전 명령 값으로 고정된 서보와 함께 프로그램이 종료됩니다. 따라서 종료하기 전에 로봇을 유휴 상태로 두는 것이 좋습니다. 유휴 모드에서 서보는 고정된 위치를 유지하지 않고 자유롭게 회전할 수 있도록 명령을 받습니다.

## Description of ROS Nodes
* **spot_micro_motion_cmd**: 로봇 제어 소프트웨어를 실행하는 메인 노드입니다. 상태 이벤트 명령과 모션 명령을 받아 서보 제어 명령을 출력합니다. 다양한 소프트웨어 설정을 위해 yaml 구성 파일을 활용합니다. 5가지 상태가 있는 상태 기계와 다음 모드 다이어그램으로 구성됩니다: 

![Spot Micro Walking](assets/state_machine.png)

기본 보행은 8단계로 구성된 걷기 스타일 보행으로, 한 번에 한쪽 다리만 스윙하고, 다리 스윙 사이에 몸을 이동하여 바닥에 남아 있는 3개의 다리로 몸의 균형을 유지합니다. 소프트웨어에서는 속도 또는 각도 명령 제한이 구현되지 않지만 역운동학 모델은 수학 오류를 방지하기 위해 삼각 영역 기능 제한을 순조롭게 수행합니다.

yaml 구성 파일은 서보 구성 사전을 포함하여 다양한 소프트웨어 구성 설정을 유지하는 데 사용됩니다. 서보는 Servo_move_keyboard 노드를 사용하여 보정할 수 있으며, 합리적인 성능을 위해서는 눈을 통한 각도 보정이면 충분합니다.

* **i2cpwm_board**: Node that controls the pca 9685 servo control board. Operates mostly under proportional control mode, but also in absolute control mode to command servos to idle

* **spot_micro_keyboard_command**: Node that sends state, motion, and rate commands to the motion control node via keyboard

* **spot_micro_joy**: Sends the same commands like the keyboard_command_node but is controlled by sensor_msgs/Joy, which are emitted by joy_node. By default it is configured for PS4 button-layout. Make sure to take a look into the [joystick control](docs/joystick_control.md) documentaion before trying.

* **lcd_monitor**: Node that displays basic state information and control values on the lcd monitor

* **spot_micro_plot**: Displays a wireframe figure of the robot via matplotlib and received state data from spot_micro_motion_cmd. This plot node can be used in lieu of the real robot for testing motions if the spot_micro_motion_cmd node is run standalone, and with the debug_mode parameter set true.

* **servo_move_keyboard**: A python node that can be used in conjuction with the i2cpwm_board node to manually command an individual servo via keyboard controls. Can be used for servo calibration to build the servo configuration dictionary.

* **spot_micro_rviz**: A node to launch RVIZ and show a visualization of the spot micro model, as well as mapping and navigational elements in the future. The `show_and_move_model_via_gui` launch file can be launched standalone to show a manually moveable spot micro model via GUI sliders. 

Note that the servo control node `i2cpwm_board` should only be commanded by one node at one time. Thus `spot_micro_motion_command` and `servo_move_keyboard` should be run exclusionary; only one should ever run at one time.

* **spot_micro_launch**: Not a node, but a launch package purely for collecting high level launch files. The launch files are for more advanced use cases such as running SLAM. 

## Additional Project Components
#### URDF Model
이 프로젝트에는 시각화를 위한 사용자 정의 stl 파일 세트와 함께 스팟 마이크로 플랫폼의 URDF 모델이 포함되어 있습니다. URDF 파일은 이 README 끝에 언급된 Florian Wilk의 저장소에서 가져온 것이며 좌표계 방향을 변경하고 내 스팟 마이크로 로봇의 치수와 일치하도록 치수를 수정했습니다. 현재 이 urdf 파일은 스팟 마이크로 모델의 RVIZ 시각화에만 **사용됩니다**. 이 URDF 모델은 로봇 형상을 완벽하게 정확하게 표현한 것으로 취급되어서는 안 되며 이 저장소의 STL 파일을 3D 프린팅에 사용해서도 안 됩니다. 대신 언급된 Thingverse 파일을 사용하세요.

URDF 모델은 특정 생성 작업을 자동화하기 위해 매크로를 사용하여 urdf 파일을 정의하는 방법인 `xacro` 파일로 정의됩니다. xacro 파일은 `spot_micro_rviz/urdf` 디렉토리에 있습니다. ROS 개발 환경을 소싱한 후 `xacro`를 실행하여 검사 또는 사용을 위해 `.xacro` 파일에서 urdf 파일을 생성할 수 있습니다.

#### TF2 Publishing and Odometry
로봇 상태 변환은 TF2를 통해 게시됩니다. 관심 있는 기본 프레임은 `base_footprint`, `base_link`, `lidar_link`입니다. `base_footprint`는 로봇 프레임 베이스의 높이가 0인 좌표계입니다. `base_link`는 로봇의 몸체 중심에 고정된 좌표계로 몸체의 움직임에 따라 움직이고 회전합니다. `lidar_link`는 설치된 LiDAR와 정렬된 좌표계입니다.

주행 거리 측정 프레임인 `odom`은 선택적으로 사용할 수 있으며 `spot_micro_motion_cmd.yaml` 파일의 구성 가능한 매개변수를 통해 활성화할 수 있습니다. 활성화된 경우 `odom`은 `base_footprint` 프레임의 상위 항목입니다. **주행 거리 측정은 매우 부정확하며 전혀 보정되지 않습니다**. 이는 로봇 속도 명령의 순수한 통합이므로 시간이 지남에 따라 오류가 무제한으로 표류됩니다. 이는 유용한 목적으로 제공됩니다.

#### SLAM
로봇 프레임에 RPLidar A1과 같은 LiDAR를 장착하면 hector_slam과 같은 추가 ROS 노드를 사용하여 SLAM을 통해 2D 매핑이 가능합니다. 본 프로젝트를 통한 SLAM 실행에 대한 자세한 내용은 [SLAM 정보](docs/slam_information.md) 문서에 설명되어 있습니다.

## Future Work
현재 소프트웨어는 외부 명령 메시지를 통해 스팟 마이크로 로봇의 기본 상태 기계 작동, 정지 상태의 방향 제어, 전진, 측면 및 요 방향의 속도 명령을 완벽하게 지원합니다.

이 프로젝트에서 내가 원하는 미래 목표는 선호도 순으로 다음과 같습니다:
1. ~~Incorporate a lidar (particularly the Slamtec RPLIDAR A1) to achieve simple 2D mapping of a room via SLAM. This may require the addition of an IMU for robot orientation sensing (for example, an Adafruit 9-DOF IMU BNO055).~~
2. 감지된 2D 환경에서 간단한 작업을 실행하도록 로봇을 안내하는 자율 동작 계획 모듈을 개발합니다. 예를 들어, 방의 경계를 탐색하고 장애물을 동적으로 피합니다.
3. 카메라나 웹캠을 통합하고 기본적인 이미지 분류를 수행하는 소프트웨어 모듈을 만듭니다. 예를 들어, 닫힌 주먹이나 열린 손바닥을 인식하고 로봇이 각각에 특정한 방식으로 반응하도록 합니다.
4. 외부 방해를 거부할 수 있는 더욱 발전된 로봇 컨트롤러를 구현합니다.

## External Links and References
* Spot Micro AI community: https://gitlab.com/custom_robots/spotmicroai

* Research paper used for inverse kinematics: 
`Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). 
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6.`

* Stanford robotics for inspiration for gait code: https://github.com/stanfordroboticsclub/StanfordQuadruped
* Spot micro URDF model copied and modified from Florian Wilk's repo
    * https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk
* List of submodules utilized:
    * ros-i2cpwmboard by bradanlane for PCA9685 support
        * https://gitlab.com/bradanlane/ros-i2cpwmboard
    * spot_micro_kinematics_python by me :) for python spot micro kinematic calculations:
        * https://github.com/mike4192/spot_micro_kinematics_python 
    * spot_micro_kinematics_cpp by me :) for c++ spot micro kinematic calculations:
        * https://github.com/mike4192/spot_micro_kinematics_cpp 

