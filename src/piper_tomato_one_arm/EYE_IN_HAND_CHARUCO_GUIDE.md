# Piper Tomato One Arm + Gemini 335 Eye-in-Hand ChArUco 가이드

이 문서는 워크스페이스에 이미 있는 `handeye_target_detection`, `handeye_dashboard`, `handeye_tf_service`, `handeye` 패키지를 사용한다.

Eye-in-hand 구성은 **Gemini 335가 로봇 손목에 장착**되고 **ChArUco 보드가 작업대에 고정**된 경우다.

최종 결과:

```text
right_gripper_base -> camera_color_optical_frame
```

## 1. 좌표계와 물리 배치

```text
right_base_link
  └─ 움직임: right_gripper_base
                   └─ Gemini 335
                       camera_color_optical_frame
                                │ 관측
                                ▼
                       고정된 calib_board
```

대시보드는 각 샘플에서 다음 TF를 조회한다.

```text
right_base_link -> right_gripper_base
camera_color_optical_frame -> calib_board
```

## 2. 준비와 안전

- 로봇 전원을 끈 상태에서 USB-CAN과 CAN 케이블을 연결한다.
- Gemini 335를 손목 브래킷에 단단히 고정한다.
- 카메라 USB 케이블이 관절에 끼이거나 당겨지지 않게 고정한다.
- 모든 자세에서 브래킷과 케이블의 충돌 가능성을 확인한다.
- ChArUco 보드를 작업대에 고정한다. 수집 중 움직이면 다시 수집한다.
- 처음에는 `auto_enable:=false`, 속도 10% 이하를 사용한다.
- 카메라 장착 위치가 바뀌면 캘리브레이션을 다시 해야 한다.

## 3. 실제 ChArUco 보드와 설정

```yaml
width: 7
height: 5
dictionary: DICT_6X6_250
charuco_board_marker_size: 0.022
charuco_board_square_size: 0.030
marker_border_bits: 1
```

`width=7`, `height=5`는 `SQUARES_X=7`, `SQUARES_Y=5`다. square는 30 mm, marker는 22 mm이며 dictionary는 `DICT_6X6_250`이다. C++ 검출기의 OpenCV 기본 border bits가 1이므로 별도 YAML 설정 없이 실제 보드와 일치한다.

## 4. Gemini/ChArUco 설정

수정 파일:

```text
/home/user/ros2_ws/src/HandEyeCalibration/handeye_target_detection/launch/pose_estimation.yaml
```

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

원본 `/top/camera/...` 토픽을 `/camera/...`로 변경해야 Gemini 기본 namespace와 맞는다.

## 5. 최초 1회 빌드

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

검출기만 다시 빌드할 때:

```bash
colcon build \
  --symlink-install \
  --packages-select handeye_target_detection \
  --event-handlers console_direct+
```

HandEye 빌드에 `--packages-up-to piper_tomato_one_arm_moveit_config`를 섞으면 로컬 MoveIt 전체와 테스트 의존성까지 빌드되어 `ros_testing` 누락으로 실패할 수 있다. HandEye와 로봇/MoveIt 빌드는 분리한다.

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

완료 후 카메라 USB를 다시 연결한다.

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

확인:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/color/camera_info --once | grep frame_id
ros2 topic echo /camera/depth/camera_info --once
```

선택된 프로파일은 color `1920x1080 RGB @ 30 FPS`, depth `1280x720 Y16 @ 30 FPS`여야 한다. ChArUco 검출은 color 영상과 **1920x1080 color CameraInfo**만 사용한다. Depth는 hand-eye pose 계산에 직접 사용하지 않지만 이후 RGB-D 운용을 위해 30 FPS로 함께 실행한다. 실제 color optical frame도 기록한다. 일반적으로 `camera_color_optical_frame`이다.

## 8. 실행 2: CAN과 Piper

```bash
ip -brief link
cd /home/user/ros2_ws/src/piper_ros
bash can_activate.sh can0 1000000
ip -details link show can0
```

`state UP`, `bitrate 1000000`을 확인한다.

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash

ros2 launch piper_tomato_one_arm_moveit_config real_robot.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  use_rviz:=true \
  command_speed_percent:=10.0
```

확인:

```bash
ros2 topic hz /joint_states_piper
ros2 run tf2_ros tf2_echo right_base_link right_gripper_base
```

손목 카메라와 케이블이 추가되었으므로 실제/RViz 자세, 관절 방향, 충돌 여유를 확인하기 전에는 자동 활성화하지 않는다.

## 9. 실행 3: ChArUco 단독 시험

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch handeye_target_detection pose_estimation.launch.py
```

`rqt_image_view`에서 `/image/detected`를 선택한다.

```bash
ros2 run rqt_image_view rqt_image_view
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame calib_board
```

정상 확인 후 단독 검출 launch를 종료한다. 대시보드 launch가 같은 검출 노드를 자동 실행한다.

## 10. 실행 4: Hand-eye 대시보드

터미널 3:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 launch handeye_dashboard handeye_dashboard.launch.py
```

이 명령은 ChArUco detector, TF service, rqt GUI를 함께 실행한다. detector를 별도로 중복 실행하지 않는다.

## 11. Eye-in-hand GUI 설정

```text
Camera-Mount-Type:   attached on robot
Camera-Frame:        camera_color_optical_frame
Object-Frame:        calib_board
Robot-Base-Frame:    right_base_link
End-Effector-Frame:  right_gripper_base
```

Camera-Frame에는 `/camera/color/camera_info`에서 확인한 실제 frame ID를 넣는다.

## 12. 샘플 수집

1. 고정 보드가 보이는 첫 자세로 천천히 이동한다.
2. 로봇과 영상이 완전히 안정될 때까지 기다린다.
3. 첫 번째 `Take a snapshot` 버튼을 누른다.
4. 서로 다른 위치와 손목 회전으로 25~40세트를 수집한다.

반드시 포함할 변화:

- 양방향 roll과 pitch
- 서로 다른 yaw
- 가까운/중간/먼 거리
- 영상 중앙과 가장자리

주의:

- 큰 손목 회전에서 USB 케이블이 당겨지지 않아야 한다.
- 움직이는 중 저장하지 않는다.
- 검출 축이 흔들리거나 사라진 샘플을 저장하지 않는다.
- 정면 자세만 반복하지 않는다.

## 13. 계산 및 결과 발행

1. 두 번째 `Get the camera/robot transform` 버튼을 누른다.
2. 계산 실패 메시지가 없는지 확인한다.
3. 네 번째 `Start publishing the TF` 버튼을 누른다.
4. 세 번째 clear 버튼은 샘플 전체를 삭제하므로 주의한다.

Eye-in-hand에서는 다음 TF가 발행된다.

```text
right_gripper_base -> camera_color_optical_frame
```

결과:

```text
/tmp/camera-robot.json
/tmp/camera-robot.txt
```

텍스트 결과 quaternion은 `[w, x, y, z]` 순서다. ROS static publisher에는 `[x, y, z, w]`로 넣는다.

## 14. 검증

발행 결과 확인:

```bash
ros2 run tf2_ros tf2_echo right_gripper_base camera_color_optical_frame
```

고정 보드의 base 기준 pose를 여러 샘플에서 계산하면 거의 일정해야 한다.

```text
T_base_board
 = T_base_gripper
 * T_gripper_camera
 * T_camera_board
```

초기 목표는 위치 잔차 5~10 mm 이하, 회전 잔차 약 1도 내외다.

## 15. URDF 또는 static TF 반영

대시보드 종료 후에도 사용하려면 결과를 영구 설정에 보존한다.

검증용 예시:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x X --y Y --z Z \
  --qx QX --qy QY --qz QZ --qw QW \
  --frame-id right_gripper_base \
  --child-frame-id camera_color_optical_frame_calibrated
```

최종 URDF에서는 보통 그리퍼와 카메라 mount/link를 fixed joint로 연결하고, Orbbec가 제공하는 mount-to-optical TF 구조를 유지한다. 동일 child frame을 URDF와 static publisher가 동시에 발행하면 안 된다.

## 16. 매번 실행하는 최소 순서

1. Gemini launch 실행
2. CAN 활성화
3. Piper real robot launch 실행
4. 카메라/조인트/TF 확인
5. handeye dashboard 실행
6. GUI에서 `attached on robot`과 네 프레임 설정
7. 25~40 샘플 수집
8. 계산 버튼
9. 발행 버튼
10. 결과 백업 및 고정 보드 pose 일관성 검증

## 17. 주요 문제 해결

### `/image/detected`가 없음

```bash
ros2 node list
ros2 topic list | grep -E 'camera|detected'
```

YAML의 토픽이 `/top/camera/...`가 아니라 `/camera/...`인지 확인한다.

### `calib_board` TF가 없음

- dictionary와 7x5 보드 설정을 확인한다.
- 조명, 초점, 보드 인쇄 상태를 확인한다.
- 실제 camera optical frame 이름을 확인한다.

### 스냅샷 TF lookup 실패

```bash
ros2 run tf2_ros tf2_echo right_base_link right_gripper_base
ros2 run tf2_ros tf2_echo camera_color_optical_frame calib_board
```

두 TF가 모두 정상이어야 한다.

### 계산 실패 또는 큰 오차

- 25개 이상의 다양한 회전 샘플을 사용한다.
- 카메라 브래킷과 고정 보드가 움직이지 않았는지 확인한다.
- 로봇 정지 후 저장한다.
- marker/square 실측 치수와 미터 단위를 확인한다.
- color 영상과 color CameraInfo 해상도를 일치시킨다.

### 결과가 수십 cm 또는 180도 틀림

- Eye-in-hand에서 반드시 `attached on robot`을 선택한다.
- 프레임 철자와 방향을 확인한다.
- quaternion 순서를 확인한다.
- optical frame에 임의 축 변환을 추가하지 않는다.

### 빌드 중 `Findros_testing.cmake` 오류

HandEye 문제가 아니라 MoveIt 테스트 의존성이 함께 선택된 것이다. 위의 HandEye 전용 `--packages-select ... --cmake-args -DBUILD_TESTING=OFF` 명령을 사용한다.
