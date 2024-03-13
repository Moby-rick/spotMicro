# Servo Calibration Guide

이 문서는 스팟 마이크로 프레임에 서보를 설치 및 보정하고 ROS 모션 명령 노드에 필요한 해당 서보 구성을 생성하는 데 대한 포괄적인 가이드를 제공합니다.
서보 구성 값을 계산하는 데 도움이 되도록 `servo_calibration_spreadsheet.ods` 라는 스프레드시트가 `docs` 디렉터리의 저장소에 포함되어 있습니다.

서보 구성 사전은 구성 파일 `spot_micro_motion_cmd.yaml` 내에 포함되어 있으며 아래와 같이 서보 구성 값을 보유합니다.
```yaml
num_servos: 12
servo_max_angle_deg: 82.5
RF_3: {num: 1, center: 306, range: 385, direction:  1, center_angle_deg:  84.0}
RF_2: {num: 2, center: 306, range: 385, direction:  1, center_angle_deg: -27.9}
RF_1: {num: 3, center: 306, range: 396, direction: -1, center_angle_deg:  -5.4}
RB_3: {num: 4, center: 306, range: 394, direction:  1, center_angle_deg:  90.4}
.
.
.
```

서보는 다리 내의 위치와 위치를 나타내는 약어와 숫자로 정의됩니다. "RF"는 오른쪽 앞, "RB"는 오른쪽 뒤, 마찬가지로 "LF"와 "LB"는 왼쪽 앞다리와 왼쪽 뒷다리에 해당합니다. 숫자 1은 링크 1(고관절)에 해당하고, 2는 링크 2(윗다리 또는 어깨 관절)에, 3은 링크 3(아래다리 또는 무릎 관절)에 해당합니다.

### Description of Servo Configuration Values
* **num_servos**: 이 spot micro platform의 경우 12로 고정되었습니다.

* **servo_max_angle_deg**: 모든 서보에 대해 단일 방향으로 허용되는 최대 명령 각도입니다. 로봇 서보의 전체 이동 범위는 일반적으로 180도(즉, 중심에서 각 방향으로 +/- 90도)입니다. 이 제한의 최대값은 일반적으로 90도이지만 서보 중심 위치의 오류, 서보 이동 가장자리 근처의 잠재적인 성능 손실을 설명하거나 기계적 제한이나 제한을 피하기 위해 이 값을 약간 제한하는 것이 유용합니다. 내 로봇에는 82.5도의 값이 작동합니다.

* **num:** 해당 서보가 연결된 PCA9685 보드의 포트(1-16번)

* **center:** PCA9685 노드가 서보 중심 위치를 나타내는 "raw" PWM 값을 수신합니다. 서보의 위치는 일반적으로 50hz 주기(20ms 기간)의 1~2ms 펄스로 명령됩니다. PCA9685 보드와 이에 상응하는 ros 노드는 12비트 PWM 값을 통해 이 펄스의 길이를 제어합니다. 총 사이클 길이가 20ms라고 가정하면 0은 펄스가 없음을 의미하고, 2^12=4096은 일정한 높은 신호를 의미하며, 예를 들어 2048은 펄스 길이가 10ms임을 의미합니다. 서보의 중심 위치는 일반적으로 약 307의 PWM 값에 해당하는 1.5ms 펄스로 배치됩니다. 이 값은 서보 키보드 이동 노드를 통해 경적을 사용하여 샘플 서보를 이동하고 중심점을 찾아 미세 조정할 수 있습니다. 서보가 반대 방향으로 동일하게 움직이는 경우(예를 들어 중심에서 +/- 90도 동일하게 이동합니다. 307의 값은 좋은 시작점이며 값은 같은 종류의 세트에 있는 모든 서보에 대해 동일해야 합니다.

* **range:** 서보의 최대 끝에서 끝까지 이동 범위에 해당하는 "raw" PWM 값입니다. 이는 구성 파일에 정의된 Servo_max_angle_deg와 관련됩니다. 예를 들어, 서보 범위 값은 +80도에 있는 서보 위치에 대한 원시 PWM 값에서 -80도에 있는 서보 위치에 대한 원시 PWM 값을 뺀 값입니다. 이 저장소의 스프레드시트는 이 범위 값을 계산합니다.

* **direction:** 1 또는 -1의 값입니다. 필요한 경우 특정 다리 관절의 좌표축에 따라 방향이나 서보 회전을 반대로 바꿉니다. 서보 교정 스프레드시트에서 계산됩니다.

* **center_angle_deg:** 다리 관절의 특정 좌표계에서 "중앙" PWM 값에 있을 때 서보의 위치에 대응하는 각도 값입니다. 서보 교정 스프레드시트에서 계산된 값입니다.


### Servo Installation
서보는 PCA 9685 제어 보드에 다음 순서로 연결되어야 합니다.
1. 오른쪽 앞 무릎
2. 오른쪽 앞 어깨
3. 오른쪽 앞 엉덩이
4. 오른쪽 무릎
5. 오른쪽 뒤 어깨
6. 오른쪽 허리 엉덩이
7. 왼쪽 무릎
8. 왼쪽 뒤 어깨
9. 왼쪽 엉덩이
10. 왼쪽 앞 무릎
11. 왼쪽 앞 어깨
12. 왼쪽 앞 엉덩이

서보에 전원이 공급되고 중앙 위치로 명령될 때 스팟 마이크로 프레임에 서보를 설치하는 것이 좋습니다. 서보가 설치된 관절은 대부분의 다리 움직임이 발생하는 위치인 "중립" 자세에 대략적으로 위치해야 합니다. 이는 일반적인 관절 명령 각도 주변에서 최대 서보 이동이 가능하도록 보장합니다. 아래 두 그림은 서보가 중앙 위치에 설치되어야 하는 관절 방향을 대략적으로 나타냅니다.

![Side View Neutral Positions](../assets/1_robot_right_links.png)
![Back View Neutral Positions](../assets/12_robot_back_overview.png)


### Commanding Individual Servos for Calibration
교정 절차를 위해 Servo_move_keyboard ROS 노드를 통해 개별 서보에 명령을 내릴 수 있습니다. 하나 이상의 서보가 PCA9685에 연결된 후 단일 서보에 명령을 내리는 단계는 다음과 같습니다.

1. 라즈베리 파이에서 i2cpwm_board 노드를 시작합니다:
`rosrun i2cpwm_board i2cpwm_board`
2. 라즈베리 파이의 다른 터미널이나 ROS가 설치되어 있고 라즈베리 파이와 통신할 수 있는 Linux 시스템에서 Servo_move_keyboard ros 노드를 실행합니다:
`rosrun servo_move_keyboard servoMoveKeyboard.py`
3. 설명 프롬프트가 나타나면 `oneServo`를 입력하여 하나의 서보 명령 노드로 들어갑니다.
4. 명령할 PCA9685 포트에 해당하는 정수 번호(예: 2)를 입력하여 명령할 서보를 선택합니다. 서보를 선택한 후 다른 모든 서보는 손으로 움직일 수 있도록 유휴(또는 프리휠) 명령을 받습니다. 다음 프롬프트가 나타납니다:

![Servo move prompt](../assets/servo_move_prompt.png) 

5. 선택된 서보의 기본 중심값(pwm = 306)을 명령하려면 `y` 키를 사용하십시오. `g` 및 `j` 키를 사용하여 서보 PWM 명령 값을 각각 1씩 줄이거나 늘리십시오. 이것은 서보를 미세한 단위로 움직입니다. 현재 PWM 값이 터미널에 인쇄됩니다. `f` 및 `k` 키를 사용하여 서보를 더 거친 증분으로 이동합니다.
    * 키의 `z` 및 `x`를 사용하여 서보에 각각 최소값과 최대값을 빠르게 명령할 수 있습니다(기본값은 83 및 520이지만 지침 프롬프트에 지정된 키를 사용하여 서보별로 업데이트할 수 있음). 최소/최대 명령에 주의하고 서보가 물리적으로 해당 위치로 이동할 수 있고 막히지 않는지 확인하십시오.
6.서보에 원하는 교정 위치를 명령하고 교정 스프레드시트에 값을 기록한 후 `q`를 눌러 서보 제어 모드를 종료합니다.
7. 다른 서보에 대해 프로세스를 반복하려면 3단계로 돌아가세요.
    
### Guide to Creating Servo Calibration Values
아래 다이어그램에 설명된 것처럼 육안으로 링크를 교정하면 적절한 기구학적 성능을 얻을 수 있습니다. 하지만 링크 1 보정을 위해 45도 각도를 측정하기 위한 보조 수단으로 스마트폰 경사계 앱도 사용했습니다. 교정을 위해 다리가 자유롭게 매달릴 수 있도록 로봇을 상자나 유사한 테스트 스탠드에 올려 놓을 것을 권장합니다.

**다리 링크는 참조점에서 참조점까지의 직선 세그먼트(예: 회전축에서 회전축까지)로 시각화되어야 합니다.** 다리 링크는 참조점에서 참조점까지의 직선 세그먼트(예: 회전축에서 회전축까지)로 시각화되어야 합니다.

아래 그림에 표시된 것처럼 Servo_calibration_spreadsheet를 사용하여 서보 교정을 구성할 수 있습니다. 일반적으로 절차는 각 위치를 두 개의 참조 위치(각도)에 연결하고 해당 위치에 해당하는 PWM 값을 기록하는 것입니다. 눈으로 쉽게 배치할 수 있도록 총 각도 0도와 90도를 사용했습니다. 기울기 값은 사용되지 않지만 모든 서보는 유사하게 계산된 기울기를 가져야 하므로 계속 관찰하는 것이 유용합니다. 이 값의 큰 불일치는 오류를 나타낼 수 있습니다.
만약 서보 3과 12에 대해 예상치 못한 값이 나왔다면, 반전된 힙 서보로 수정된 모델을 프린트했는지 확인해보세요. 이를 확인하려면 본체 내부를 살펴보세요. 4개의 볼 베어링이 보이면 원래 디자인이 있는 것입니다. 2개의 볼 베어링과 2개의 서버 혼이 보인다면, 서보 3과 12가 반전된 것입니다.

![Servo Calibration Spreadsheet](../assets/servo_calibration_spreadsheet.png)

#### 오른쪽 다리 링크 2 및 3 보정

**다음 단계에서는 일관성이 정확성보다 더 중요합니다.**

로봇의 오른쪽부터 시작하여 모든 다리에 대한 링크 2와 3을 교정하는 것부터 시작하세요. 아래 그림은 0도에 위치한 오른쪽 다리 링크의 좌표계를 나타내며 양의 각도 방향과 음의 각도 방향을 정의합니다

![Right side zero degree positions](../assets/2_right_straight_links.png)

아래 그림에 예시된 것처럼 링크 3의 각도는 링크 2를 기준으로 합니다.
![Example right side link 3 angles ](../assets/3_right_link_angles_example.png)

링크 2(링크 3의 방향 무시)부터 시작하여 링크 2에 0도와 -90도 위치를 지정하고 스프레드시트에 해당 PWM 값을 기록해 둡니다. 이 두 위치는 다음 두 그림에 나와 있습니다.

![Right side link 2 step 1 ](../assets/4_right_link2_config_step_1.png)
![Right side link 2 step 2 ](../assets/4_right_link2_config_step_2.png)

다음으로 링크 3으로 이동합니다. 참조 방향을 쉽게 시각화할 수 있도록 링크 2를 수직 방향으로 수동으로 배치합니다. 명령은 3을 0 및 +90도 위치로 연결하고 해당하는 PWM 값을 기록합니다. 링크 3의 두 위치는 다음 두 그림에 표시되어 있습니다.

![Right side link 3 step 1 ](../assets/5_right_link3_config_step_1.png)
![Right side link 3 step 2 ](../assets/6_right_link3_config_step_2.png)

다른 오른쪽 다리에도 이 과정을 반복합니다.

#### Left Side Legs Links 2 and 3 Calibration
다음으로 로봇의 왼쪽 다리에 대해 이 과정을 반복합니다. 왼쪽 다리의 좌표계는 양의 각도와 음의 각도에 대해 서로 다른 방향을 가지고 있습니다 . 이는 완전성을 위해 아래 그림에 설명되어 있습니다.

![Left side overview ](../assets/7_robot_left_overview.png)

왼쪽 다리의 링크 2에 대해 보정 과정을 반복합니다. 링크 2를 0도 및 +90도 위치에 배치하고 스프레드시트에 PWM 값을 기록합니다. 두 위치는 다음 두 그림에 나와 있습니다.

![Left side link 2 step 1 ](../assets/8_left_link2_config_step_1.png)
![Left side link 2 step 2 ](../assets/9_left_link2_config_step_2.png)

왼쪽 다리의 링크 3에 대해 보정 과정을 반복합니다. 링크 2를 수직으로 수동으로 배치한 다음 링크 3을 0도와 -90도에 배치하고 해당하는 PWM 값을 스프레드시트에 기록합니다. 두 위치는 아래에 나와 있습니다.

![Left side link 3 step 1 ](../assets/10_left_link3_config_step_1.png)
![Left side link 3 step 2 ](../assets/11_left_link3_config_step_2.png)

양쪽 왼쪽 다리에도 이 과정을 반복합니다.

#### Left and Right Legs Link 1 Calibration

마지막으로, 비슷한 패턴으로 왼쪽 다리와 오른쪽 다리에 대한 링크 1을 보정합니다. 왼쪽과 오른쪽 다리의 링크 1에 대한 좌표계는 아래 그림과 같습니다.

![Back view Coordinate Systems link 1 ](../assets/12_robot_back_overview.png)
![Back view Coordinate Systems link 1 ](../assets/13_robot_back_angle_directions.png)

오른쪽 다리부터 시작하여 1을 0도 및 -45도로 연결하고 해당 PWM 값을 스프레드시트에 기록합니다. 기계적 한계로 인해 90도 대신 45도의 기준값을 사용합니다. 왼쪽 다리에도 이 과정을 반복합니다. 이러한 위치는 아래 4개의 그림에 나와 있습니다.

![Right leg link 1 step 1](../assets/14_right_link1_config_step_1.png)
![Right leg link 1 step 2](../assets/15_right_link1_config_step_2.png)

![Left leg link 1 step 1](../assets/16_left_link1_config_step_1.png)
![Left leg link 1 step 2](../assets/17_left_link1_config_step_2.png)

이것으로 다리 서보 교정 프로세스가 완료됩니다. 서보 교정 스프레드시트에서 굵은 값을 가져와 `spot_micro_motion_cmd.yaml` 구성 파일의 서보 구성 사전에 복사합니다. 이 파일은 catkin 작업 공간의 다음 하위 폴더에 있습니다: ```src/spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml```.
