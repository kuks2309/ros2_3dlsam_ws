# rtab_map Code Updates

> 최신순 누적 기록

## 2026-02-18

### 13:32

- **수정** `launch/rtabmap_2d_lidar_rgbd.launch.py` - use_camera launch argument 추가 (false: LiDAR only, 카메라 subscribe/remapping 조건 분기)
- **수정** `launch/rtabmap_2d_lidar_rgbd_localization.launch.py` - use_camera launch argument 추가 (false: LiDAR only, 카메라 subscribe/remapping 조건 분기)

### 13:24

- **수정** `launch/rtabmap_2d_lidar_rgbd.launch.py` - static_tf_base_to_camera + camera_z 제거 (TF를 tm_nav_tool 카메라 lifecycle로 이동)
- **수정** `launch/rtabmap_2d_lidar_rgbd_localization.launch.py` - static_tf_base_to_camera + camera_z 제거 (TF를 tm_nav_tool 카메라 lifecycle로 이동)

### 13:20

- **수정** `launch/rtabmap_2d_lidar_rgbd_localization.launch.py` - map_server + lifecycle_manager 추가 (저장된 map.yaml 즉시 발행), map.yaml 존재 시 RGBD/CreateOccupancyGrid 비활성화
- **수정** `rviz/rtabmap_2d_lidar_rgbd.rviz` - /rtabmap/map, /rtabmap/map_updates Durability Policy: Volatile → Transient Local

---

## 2026-02-17

### 20:49

- **수정** `launch/rtabmap_2d_lidar_rgbd.launch.py` - base_link→camera_link static TF 추가 (camera_z launch arg, 기본값 0.85m)
- **수정** `launch/rtabmap_2d_lidar_rgbd_localization.launch.py` - base_link→camera_link static TF 추가 (camera_z launch arg, 기본값 0.85m)

### 20:41

- **추가** `launch/rtabmap_2d_lidar_rgbd.launch.py` - 2D LiDAR(주) + RGB-D(보조) mapping launch (ICP odometry, Reg/Strategy=1, Grid/Sensor=2, Grid/3D=true)
- **추가** `launch/rtabmap_2d_lidar_rgbd_localization.launch.py` - 2D LiDAR(주) + RGB-D(보조) localization launch
- **추가** `rviz/rtabmap_2d_lidar_rgbd.rviz` - 전용 rviz config (LaserScan, CloudMap RGB8, Map, TF, Odometry, RGB Image)

### 20:01

- **수정** `launch/rtabmap_dual_lidar.launch.py` - range_max: 5.5 → 40.0 (SICK 센서 사양 반영)
- **수정** `launch/rtabmap_dual_lidar_localization.launch.py` - range_max: 5.5 → 40.0 (SICK 센서 사양 반영)

