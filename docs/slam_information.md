# SLAM Information
이 문서는 이 프로젝트를 통해 스팟 마이크로 4족보행 로봇에서 SLAM을 실행하기 위한 추가 정보를 제공합니다.

* [Required Setup](#required-setup)
* [Generating a Map](#generating-a-map-frames)
* [Saving a Map](#saving-a-map)
* [Recording a Data Log](#recording-a-data-log)
* [Reprocessing Scan Data Through Log Playback](#reprocessing-scan-data-through-log-playback)

![Walking and Slam](../assets/walking_and_slam.gif)
Example of robot walking and mapping an environment.

## Required Setup
SLAM을 위해서는 두 가지 전제 단계가 필요합니다:
1. 추가 ROS 패키지 설치
2. 라이다가 연결된 ROS 패키지의 USB 포트 권한 상승

두 개의 ROS 패키지를 추가로 설치해야 하며, 이러한 패키지는 Raspberry Pi에 설치해야 합니다. 데이터 재처리와 같은 용도로 리눅스 호스트 시스템에 설치하는 것도 좋습니다. 필요한 패키지는 hector_slam 및 rplidar_ros이며, 다음 명령을 사용하여 설치할 수 있습니다:
```
sudo apt-get install ros-kinetic-rplidar-ros
sudo apt-get install ros-kinetic-hector-slam
```
위의 명령이 작동하지 않을 경우 소스 목록 설정에 대한 자세한 내용은 다음 [website](http://wiki.ros.org/kinetic/Installation/Ubuntu)를 참조하십시오.

RPLidar가 연결된 USB 포트에 고급 사용 권한을 부여해야 합니다. RPLidar를 RPi의 USB 포트 중 하나에 연결한 후 다음 명령을 사용하여 연결된 USB 포트를 찾습니다.
`ls -l /dev |grep ttyUSB`
다음 명령을 사용하여 포트를 승인합니다. 포트가 USB0인 경우 `USB0`:
`sudo chmod 666 /dev/ttyUSB0`

이 단계는 RPi가 전원을 켤 때마다 수행될 수도 있고 수행될 필요도 없을 수도 있습니다. 적어도 Ubiquity Robotics RPi Ubuntu 이미지를 ROS와 함께 사용할 때는 영구적으로 적용되는 것으로 보입니다. rplidar_ros 패키지와 함께 사용할 RPLidar A1을 설정하는 방법에 대한 자세한 내용은 다음 [page](https://github.com/robopeak/rplidar_ros/wiki)를 참조하십시오.

RPLidar A1은 USB를 통해 직렬 보드에 전원이 공급되는 USB 포트에 연결되면 회전하기 시작합니다. SDK를 통해 사용하지 않을 때 회전하지 않도록 라이다를 중지하도록 명령할 수 있지만 이 기능은 아직 이 프로젝트에 구현되지 않았습니다.

이 프로젝트는 RPLidar A1을 가정합니다. 다른 Lidar를 사용하는 경우 다른 Lidar 드라이버 ROS 패키지가 필요합니다.


## Generating a Map
라이다로 매핑할 때 로봇 동작을 위해 trot gait 을 사용하는 것이 좋습니다. 8단계 보행보다 안정성은 떨어지지만 약간 더 부드럽기 때문입니다.

적어도 두 개의 터미널 창을 열고 하나의 ssh를 라즈베리 파이에 연결합니다. 편의상 `tmux`와 같은 터미널 멀티플렉서를 사용하는 것을 추천합니다. 각 터미널에서 다음 런칭 파일을 시작합니다:
* `roslaunch spot_micro_launch motion_control_and_hector_slam.launch`: Raspberry Pi에서 실행합니다. i2c_pwm 보드 노드, 로봇의 동작 제어 노드, 헥터_매핑 및 라이다 드라이버 노드(rplidar_ros)를 시작합니다.
* `roslaunch spot_micro_launch keyboard_control_and_rviz rviz_slam:=true` 로컬 기기에서 실행합니다. 매핑 프로세스를 표시하는 구성으로 rviz 뿐만 아니라 터미널의 스팟 마이크로 로봇에 키보드 명령을 실행하기 위한 키보드 명령 노드를 시작합니다.

모든 것이 실행된 후에는 키보드 명령 노드를 통해 로봇이 수동으로 환경(예: 방 또는 아파트)을 돌아다니도록 지시할 수 있으며, RVIZ를 통해 지도가 생성되고 로봇의 현재 및 과거 위치와 함께 실시간으로 표시됩니다.

## Saving a Map
미래에 내비게이션 용도로 사용할 수 있도록 지도를 저장하려면 RVIZ에 만족스러운 지도가 표시될 때까지 로봇을 실행합니다. 모든 노드를 계속 실행하고 RPi 또는 로컬 리눅스 호스트의 ros 환경에서 다른 터미널을 열고 다음 명령을 실행합니다:
`rosrun map_server map_saver -f my_map_filename`
이렇게 하면 명령이 실행된 동일한 디렉터리에 두 개의 파일(my_map_filename.pgm 및 my_map_filename.yaml)이 생성됩니다. `my_map_filename` 자리에 다른 사용자 지정 이름을 쓸 수 있습니다. rosmap_server에 대한 자세한 내용은 다음 [website](http://wiki.ros.org/map_server)를 참조하십시오.

다른 유형의 맵을 생성할 수 있는데, 이 맵은 순수하게 이미지이며 탐색 목적으로 유용하지 않습니다. 이 맵은 ros 환경에서 다음 명령을 실행하여 생성할 수 있습니다:
`rostopic pub syscommand std_msgs/String "savegeotiff"`


## Recording a Data Log
데이터 로그를 기록하는 것은 나중에 데이터를 재생하고 검사 또는 분석할 때뿐만 아니라 다음 섹션에서 설명하는 데이터를 재처리할 때도 유용합니다.

데이터 로그를 기록하고 로그 파일을 저장할 새 디렉토리를 만듭니다:
```
mkdir ~/bagfiles
cd ~/bagfiles
```
그리고 다음 명령으로 모든 로스토픽을 기록합니다:
`rosbag record -a`

로그 파일은 약 10분 동안 작동할 때마다 1GB 정도의 크기로 제공됩니다.

## Reprocessing Scan Data Through Log Playback
데이터 로그를 재생하여 데이터를 재처리할 수 있습니다. 라이다 스캔 데이터로 로그 파일을 재생하여 지도를 다시 생성하고, 다른 매핑 설정이나 전혀 다른 매핑 패키지를 사용할 수 있는 것이 한 가지 활용 사례입니다.

예를 들어 헥터 슬램을 통해 기록된 데이터 세트를 재생하는 단계는 다음과 같습니다:
1. `rosbag play <your bagfile>`과 같은 명령어를 사용하여 로그를 재생합니다. 재생할 토픽을 지정하기 위해 `--topics <topic1_to_play> <topic2_to_play> ...` 과 같은 추가적인 명령줄 인수를 추가할 수 있습니다.
2. hector slam node와 rviz를 시작하여 매핑 과정을 시각화합니다. 이는 다음 두 가지 launch 명령을 통해 수행할 수 있습니다:
`roslaunch spot_micro_rviz slam.launch`
`roslaunch spot_micro_launch motion_control_and_hector_slam.launch run_post_proc:=true`


