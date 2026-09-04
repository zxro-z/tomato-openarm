# Piper Tomato One Arm + Gemini 335 Eye-to-Hand ChArUco 가이드

이 문서는 이 워크스페이스에 이미 있는 다음 패키지를 기준으로 한다.

```text
HandEyeCalibration/handeye_target_detection  # ChArUco 검출 및 TF 발행
HandEyeCalibration/handeye_dashboard         # 샘플 수집, 계산, 결과 발행
HandEyeCalibration/handeye_tf_service        # TF 조회/발행 서비스
HandEyeCalibration/handeye                   # AX=XB 계산기
```

Eye-to-hand 구성은 **Gemini 335가 로봇 외부에 고정**되고 **ChArUco 보드가 그리퍼에 고정**된 경우다.

최종 결과:

```text
right_base_link -> camera_color_optical_frame
```

## 1. 좌표계와 물리 배치

```text
고정: right_base_link                 고정: Gemini 335
       │                              camera_color_optical_frame
       └─ 움직임: right_gripper_base              │
                       │                          │ 관측
                       └─ ChArUco 보드 ───────────┘
```

대시보드는 각 샘플에서 다음 TF를 조회한다.

```text
right_base_link -> right_gripper_base
camera_color_optical_frame -> calib_board
```

## 2. 준비와 안전

- 로봇 전원을 끈 상태에서 USB-CAN과 로봇 CAN을 연결한다.
- Gemini 335를 USB 3.x 포트에 연결하고 외부 프레임에 단단히 고정한다.
- 캘리브레이션 중 카메라가 움직이면 전체 데이터를 다시 수집한다.
- ChArUco 보드를 그리퍼에 단단히 고정한다.
- 보드와 브래킷이 주변 구조물에 충돌하지 않는지 확인한다.
- 처음에는 `auto_enable:=false`, 속도 10% 이하를 사용한다.
- 비상 정지를 확인하고 사람은 작업 반경 밖에 있어야 한다.

## 3. 실제 ChArUco 보드 규격

이 시스템에서 사용하는 실제 보드 설정:

```yaml
width: 7
height: 5
dictionary: DICT_6X6_250
charuco_board_marker_size: 0.022
charuco_board_square_size: 0.030
marker_border_bits: 1
```

`width=7`, `height=5`는 각각 `SQUARES_X=7`, `SQUARES_Y=5`를 뜻한다. 길이 단위는 미터이며 square는 30 mm, marker는 22 mm다. `handeye_target_detection`의 OpenCV `DetectorParameters` 기본 marker border bits가 1이므로 별도 YAML 파라미터 없이 실제 보드와 일치한다.

## 4. ChArUco/Gemini 설정 수정

다음 파일을 연다.

```text
/home/user/ros2_ws/src/HandEyeCalibration/handeye_target_detection/launch/pose_estimation.yaml
```

Gemini를 `camera_name:=camera`로 실행할 때 설정은 다음과 같아야 한다.

```yaml
pose_estimation:
  ros__parameters:
    pattern: CHARUCO
    image_topic: /camera/color/image_raw
    camera_info_topic: /camera/color/camera_info
    publish_image_topic: /image/detected
    width: 7
    height: 5
    dictionary: DICT_6X6_250
    charuco_board_marker_size: 0.022
    charuco_board_square_size: 0.030
```

현재 원본 설정의 `/top/camera/...` 토픽은 이 구성에 맞지 않으므로 `/camera/...`로 바꿔야 한다.

## 5. 최초 1회 빌드

HandEyeCalibration 스택만 분리해서 빌드하는 권장 명령:

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    baldor \
    criutils \
    handeye \
    handeye_msgs \
    handeye_tf_service \
    handeye_target_detection \
    handeye_dashboard \
  --cmake-args -DBUILD_TESTING=OFF \
  --event-handlers console_direct+

source install/setup.bash
```

검출기만 다시 빌드할 때는 사용자가 제시한 다음 명령이 맞다.

```bash
colcon build \
  --symlink-install \
  --packages-select handeye_target_detection \
  --event-handlers console_direct+
```

단, 이 명령은 검출기만 빌드하며 대시보드와 계산 패키지를 새로 빌드하지 않는다.

`--packages-up-to piper_tomato_one_arm_moveit_config`를 위 명령에 섞지 않는다. 로컬 MoveIt 전체와 테스트 패키지까지 빌드되어 `ros_testing` 누락으로 실패할 수 있다. 로봇/MoveIt은 이미 설치된 결과를 사용하고, 수정 후 다시 빌드해야 할 때 별도 터미널과 별도 명령으로 처리한다.

모든 새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
```

## 6. 최초 1회 Orbbec udev 규칙

```bash
cd /home/user/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

완료 후 카메라 USB를 분리했다가 다시 연결한다.

## 7. 실행 1: Gemini 335

터미널 1:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera \
  enable_color:=true \
  color_width:=1920 \
  color_height:=1080 \
  color_fps:=30 \
  enable_depth:=true \
  depth_width:=1280 \
  depth_height:=720 \
  depth_fps:=30
```

다른 터미널에서 확인한다.

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/color/camera_info --once | grep frame_id
ros2 topic echo /camera/depth/camera_info --once
```

검출 영상:

```bash
ros2 run rqt_image_view rqt_image_view
```


선택된 프로파일은 color `1920x1080 RGB @ 30 FPS`, depth `1280x720 Y16 @ 30 FPS`여야 한다. ChArUco pose는 color 영상과 **1920x1080 color CameraInfo**를 사용한다. Depth 영상은 hand-eye 계산에 직접 사용하지 않지만 이후 RGB-D 운용과 검증을 위해 같은 30 FPS로 실행한다. `frame_id`가 `camera_color_optical_frame`인지 확인하고, 다르면 GUI에 실제 출력값을 입력한다.

## 8. 실행 2: CAN과 Piper

로봇 전원을 켜고 USB-CAN을 연결한 뒤:

```bash
ip -brief link
cd /home/user/ros2_ws/src/piper_ros
bash can_activate.sh can0 1000000
ip -details link show can0
```

`state UP`, `bitrate 1000000`을 확인한다.

로봇 영점 맞추기
rviz 킬때 로봇 자체를 조금 들고 키기 
-> joint 란가서 모든 것을 0으로 세팅

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch piper_tomato_one_arm_moveit_config real_robot.launch.py \
  can_port:=can0 \
  auto_enable:=true \
  use_rviz:=true \
  command_speed_percent:=10.0
```

확인:

```bash
ros2 topic hz /joint_states_piper
ros2 run tf2_ros tf2_echo right_base_link right_gripper_base
```

그리퍼 제어 
```bash
cd && cd piper_susam_ws/susam
condaon
conda activate piper_susam_opencv_version_up
cd robot_example
python3 piper_ctrl_gripper.py 
```

true, false 
```bash
  source /opt/ros/humble/setup.bash
  source /home/user/ros2_ws/install/setup.bash

  비활성화:

  ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: false}"

  다시 활성화:

  ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: true}"
  
    # 비활성화
  ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: false}"

  # 활성화
  ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: true}"

```

로봇을 움직일 때 TF가 함께 변해야 한다. 실제 자세와 RViz 자세 및 모든 관절 방향이 일치하기 전에는 `auto_enable:=true`를 사용하지 않는다.

## 9. 실행 3: ChArUco 검출 단독 시험

대시보드를 실행하기 전에 검출만 시험하려면 터미널 3에서:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch handeye_target_detection pose_estimation.launch.py
```

검출 영상:

```bash
ros2 run rqt_image_view rqt_image_view
```

`/image/detected`를 선택한다. 보드에 좌표축이 표시되어야 한다.

TF 확인:

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame calib_board
```

값이 안정적으로 출력되는지 확인한 후 단독 검출 launch를 `Ctrl+C`로 종료한다. 다음 대시보드 launch가 검출기를 다시 자동 실행하므로 두 개를 동시에 실행하지 않는다.

## 10. 실행 4: Hand-eye 대시보드

터미널 3:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch handeye_dashboard handeye_dashboard.launch.py
```

이 launch는 자동으로 다음을 실행한다.

- `handeye_target_detection/pose_estimation.launch.py`
- `handeye_tf_service/handeye_tf_server`
- hand-eye rqt GUI

따라서 target detector를 별도 터미널에서 중복 실행하지 않는다.

## 11. Eye-to-hand GUI 설정

대시보드에 정확히 다음을 입력한다.

```text
Camera-Mount-Type:   fixed beside robot
Camera-Frame:        camera_color_optical_frame
Object-Frame:        calib_board
Robot-Base-Frame:    right_base_link
End-Effector-Frame:  right_gripper_base
```

Camera-Frame은 `/camera/color/camera_info`에서 확인한 실제 `frame_id`를 사용한다.

## 12. 샘플 수집

1. 그리퍼의 보드가 카메라에 잘 보이는 자세로 이동한다.
2. 로봇이 완전히 정지하고 `/image/detected`의 축이 안정될 때까지 기다린다.
3. 대시보드 첫 번째 카메라 버튼 `Take a snapshot`을 누른다.
4. 자세를 변경하고 25~40회 반복한다.

샘플에는 다음을 모두 포함한다.

- 가까운/중간/먼 거리
- 화면 중앙과 가장자리
- 서로 다른 roll, pitch, yaw
- 보드가 충분히 크게 보이는 자세

피해야 할 것:

- 위치만 바꾸고 회전하지 않는 데이터
- 거의 같은 자세 반복
- 로봇이 움직이는 중 저장
- 보드 일부만 보이거나 검출 축이 튀는 상태
- 과노출, 반사, 모션 블러

## 13. 계산과 결과 발행

샘플 수집 후:

1. 두 번째 버튼 `Get the camera/robot transform`을 누른다.
2. 계산 실패 메시지가 없는지 확인한다.
3. 네 번째 로봇 버튼 `Start publishing the TF`를 누른다.
4. 세 번째 지우기 버튼은 모든 샘플을 삭제하므로 주의한다.

Eye-to-hand에서 대시보드는 결과를 다음 TF로 발행한다.

```text
right_base_link -> camera_color_optical_frame
```

결과 파일:

```text
/tmp/camera-robot.json
/tmp/camera-robot.txt
```

`camera-robot.txt`의 quaternion 순서는 다음이다.

```text
[w, x, y, z]
```

ROS static publisher에는 `[x, y, z, w]`로 순서를 바꿔 넣어야 한다.

## 14. 검증

결과 TF 확인:

```bash
ros2 run tf2_ros tf2_echo right_base_link camera_color_optical_frame
```

보드가 그리퍼에 고정되어 있으므로 여러 자세에서 TF 체인을 이용한 보드 위치와 로봇 그리퍼 기반 보드 위치가 일관되어야 한다. 권장 초기 목표는 위치 잔차 5~10 mm 이하, 회전 잔차 약 1도 내외다.

오차가 크면 다음을 점검한다.

- 보드 marker/square 실측 치수
- `/camera/color/image_raw`와 color CameraInfo 해상도 일치
- 카메라 또는 보드의 기계적 움직임
- 회전 다양성 부족
- 잘못 검출된 샘플
- TF 시간차와 움직이는 중 저장

## 15. 재사용을 위한 static TF

대시보드를 종료하면 서비스가 발행하던 TF도 사라진다. `/tmp/camera-robot.txt` 값을 별도 YAML/URDF에 보존한다.

예시 형식:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x X --y Y --z Z \
  --qx QX --qy QY --qz QZ --qw QW \
  --frame-id right_base_link \
  --child-frame-id camera_color_optical_frame_calibrated
```

Orbbec 드라이버가 같은 child frame을 다른 parent로 이미 발행하는 경우 TF 충돌이 생길 수 있다. 검증 중에는 `_calibrated` 이름을 사용하고, 최종 통합 시 카메라 루트 링크와 optical frame의 기존 TF 구조를 확인해 연결한다.

## 16. 매번 실행하는 최소 순서

1. Gemini launch 실행
2. CAN `can0` 활성화
3. Piper real robot launch 실행
4. 카메라/조인트/TF 확인
5. `handeye_dashboard.launch.py` 실행
6. GUI에서 `fixed beside robot`과 네 프레임 입력
7. 25~40 샘플 수집
8. 계산 버튼
9. 발행 버튼
10. `/tmp/camera-robot.txt` 백업 및 검증

## 17. 주요 문제 해결

### `/image/detected`가 없음

```bash
ros2 node list
ros2 topic list | grep -E 'camera|detected'
```

YAML의 `/top/camera/...`가 `/camera/...`로 변경되었는지 확인한다.

### `calib_board` TF가 없음

- 보드 dictionary, width, height를 확인한다.
- marker/square 크기는 검출 여부보다 pose 축척에 영향을 준다.
- 보드 조명과 초점을 확인한다.

### 스냅샷 값이 0 또는 TF lookup 실패

```bash
ros2 run tf2_ros tf2_echo right_base_link right_gripper_base
ros2 run tf2_ros tf2_echo camera_color_optical_frame calib_board
```

두 명령이 모두 정상이어야 한다.

### 계산 실패

- 최소 개수만 채우지 말고 25개 이상 수집한다.
- 손목 회전이 다양한지 확인한다.
- clear 후 더 다양한 샘플로 다시 수집한다.

### 빌드 중 `Findros_testing.cmake` 오류

ChArUco 또는 HandEye 코드 오류가 아니라 로컬 MoveIt 테스트 의존성이 끌려온 것이다. 이 문서의 HandEye 전용 `--packages-select ... --cmake-args -DBUILD_TESTING=OFF` 명령으로 다시 빌드한다.

### 결과가 수십 cm 또는 180도 틀림

- Eye-to-hand에서 반드시 `fixed beside robot`을 선택한다.
- 프레임 방향과 철자를 확인한다.
- mm와 m를 혼용하지 않는다.
- 결과 파일 quaternion `[w,x,y,z]`를 ROS 인자 `[x,y,z,w]`로 변환한다.
