# Piper Tomato One Arm + Gemini 335 ChArUco 캘리브레이션 안내

이 워크스페이스에서는 별도의 ChArUco 검출기를 새로 만들지 않고 다음 기존 패키지를 사용한다.

```text
handeye_target_detection  ChArUco 검출, 검출 영상 및 calib_board TF 발행
handeye_tf_service        TF 조회 및 계산 결과 TF 발행
handeye_dashboard         GUI 샘플 수집, AX=XB 계산, 결과 저장
handeye                   실제 hand-eye 계산기
```

## 설치 방식에 맞는 문서 선택

### 카메라가 로봇 외부에 고정된 경우

Eye-to-hand 방식이다. ChArUco 보드를 그리퍼에 고정하고 다음 문서를 따른다.

- [EYE_TO_HAND_CHARUCO_GUIDE.md](./EYE_TO_HAND_CHARUCO_GUIDE.md)

GUI 설정:

```text
Camera-Mount-Type:   fixed beside robot
Camera-Frame:        camera_color_optical_frame
Object-Frame:        calib_board
Robot-Base-Frame:    right_base_link
End-Effector-Frame:  right_gripper_base
```

결과:

```text
right_base_link -> camera_color_optical_frame
```

### 카메라가 로봇 손목에 장착된 경우

Eye-in-hand 방식이다. ChArUco 보드를 작업대에 고정하고 다음 문서를 따른다.

- [EYE_IN_HAND_CHARUCO_GUIDE.md](./EYE_IN_HAND_CHARUCO_GUIDE.md)

GUI 설정:

```text
Camera-Mount-Type:   attached on robot
Camera-Frame:        camera_color_optical_frame
Object-Frame:        calib_board
Robot-Base-Frame:    right_base_link
End-Effector-Frame:  right_gripper_base
```

결과:

```text
right_gripper_base -> camera_color_optical_frame
```

## 공통 실행 구조

```text
Orbbec Gemini 335
  └─ /camera/color/image_raw
  └─ /camera/color/camera_info
            │
            ▼
handeye_target_detection
  └─ /image/detected
  └─ camera_color_optical_frame -> calib_board

Piper driver + robot_state_publisher
  └─ /joint_states_piper
  └─ right_base_link -> right_gripper_base

두 TF
  └─ handeye_dashboard에서 snapshot
  └─ handeye 계산
  └─ /tmp/camera-robot.json
  └─ /tmp/camera-robot.txt
```

## 중요 설정 파일

```text
/home/user/ros2_ws/src/HandEyeCalibration/handeye_target_detection/launch/pose_estimation.yaml
```

Gemini 기본 namespace를 사용할 때:

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

실제 보드는 `SQUARES_X=7`, `SQUARES_Y=5`, square 30 mm, marker 22 mm, `DICT_6X6_250`, border bits 1이다. 검출기의 OpenCV 기본 border bits도 1이다.

## 카메라 프로파일

실제 운용 프로파일은 다음과 같다.

```text
Color: 1920x1080 RGB @ 30 FPS
Depth: 1280x720 Y16 @ 30 FPS
```

Gemini 실행 시 다음 인자를 사용한다.

```bash
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

ChArUco 검출과 pose 계산은 `/camera/color/image_raw` 및 동일한 1920x1080 프로파일의 `/camera/color/camera_info`를 사용한다. Depth Y16 영상은 hand-eye 계산에는 직접 사용하지 않는다.

## HandEye 스택 빌드 명령

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

이 명령은 실제로 검증된 HandEye 전용 빌드 명령이다. `--packages-up-to piper_tomato_one_arm_moveit_config`를 함께 사용하면 로컬 MoveIt 전체와 테스트 의존성까지 선택되어 `ros_testing` 누락 오류가 발생할 수 있으므로 로봇/MoveIt 빌드와 분리한다.

## 공통 실행 순서

1. `orbbec_camera`로 Gemini 335를 실행한다.
2. `can_activate.sh can0 1000000`으로 CAN을 활성화한다.
3. `piper_tomato_one_arm_moveit_config real_robot.launch.py`를 실행한다.
4. 카메라 토픽과 두 필수 TF를 확인한다.
5. `handeye_dashboard handeye_dashboard.launch.py`를 실행한다.
6. 설치 방식에 맞는 GUI mount type을 선택한다.
7. 다양한 위치와 회전에서 25~40개 snapshot을 수집한다.
8. 계산 버튼과 TF 발행 버튼을 누른다.
9. `/tmp/camera-robot.txt`를 백업하고 결과를 검증한다.

구체적인 명령, 안전 절차, 검출 단독 시험, 샘플 수집 및 문제 해결은 위의 Eye-to-hand 또는 Eye-in-hand 상세 문서를 따른다.
