# robot_description

Eclipse 로봇(base / arm / gripper / camera_tower)의 URDF/xacro, 메쉬, ros2_control 하드웨어 정의.

## 사용법

```bash
# 컨트롤러 없이 모델만 시각화
ros2 launch robot_description display.launch.py
```

컨트롤러/하드웨어와 함께 실행하려면 `robot_bringup` 참고.

## 추가/수정 시 참고

- 새 파츠 추가 시 패턴: 서브 xacro 분리 → `urdf/eclipse.xacro`에서 include + 고정 joint로 연결
- **camera_tower 마운트 오프셋**(`eclipse.xacro`의 `base_to_camera_tower` joint origin)은 아직 가조립 값 — 실측 후 갱신 필요
- **camera_tower 다이나믹셀 Profile Velocity/Acceleration**(`ros2_control/eclipse_ros2_control.xacro`)은 arm 값을 그대로 가져온 placeholder — 실제 튜닝 필요
- 새 링크/조인트 추가 시 메쉬는 `meshes/<서브시스템>/`에 STL로 추가
