cmake_minimum_required(VERSION 3.22)
project(openarm_baseline)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/cube_pnp_scene.py
  scripts/legacy_eval_scene.py
  scripts/openarm_moveit_smoke_test.py
  scripts/head_camera_tf.py
  scripts/baseline_a_single_view.py
  scripts/baseline_a_free_orientation_diagnostic.py
  scripts/camera_visual_markers.py
  scripts/baseline_a_grasp_transform_calibration.py
  scripts/baseline_a_calibration_home.py
  scripts/baseline_a_contact_grasp.py
  scripts/baseline_a_move_to_camera.py
  scripts/baseline_a_startup_audit.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
