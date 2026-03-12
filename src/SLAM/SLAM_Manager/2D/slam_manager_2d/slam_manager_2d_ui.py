#!/usr/bin/env python3
"""
SLAM Manager UI - PyQt5 UI class for SLAM Manager

This module handles:
- UI event handling and button connections
- ROS Bag playback control
- Launch file auto-detection
- Button state management
"""

import os
import sys
import signal
import time
import subprocess
import math
from pathlib import Path
from datetime import datetime

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from ament_index_python.packages import get_package_share_directory


def get_workspace_path():
    """Detect workspace path from slam_manager_2d package installation"""
    try:
        # Get slam_manager_2d package path: /path/to/ws/install/slam_manager_2d/share/slam_manager_2d
        pkg_path = get_package_share_directory('slam_manager_2d')
        # Extract workspace: /path/to/ws
        workspace = Path(pkg_path).parents[3]
        return workspace
    except Exception:
        # Fallback to home directory
        return Path.home()


class SlamManagerUI(QtWidgets.QMainWindow):
    """PyQt5 Main Window for SLAM Manager"""

    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = Path(__file__).parent / 'ui' / 'slam_manager_2d.ui'
        uic.loadUi(ui_file, self)

        # ROS2 node (will be set later)
        self.node = None

        # Workspace path (auto-detected from package installation)
        self.workspace_path = get_workspace_path()

        # ROS Bag state
        self.bag_process = None
        self.bag_paused = False
        self.bag_duration = 0.0
        self.bag_start_time = None

        # Connect buttons - Data Source tab
        self.btnStartLivox.clicked.connect(self.on_start_livox)
        self.btnStopLivox.clicked.connect(self.on_stop_livox)
        self.btnBrowseBag.clicked.connect(self.on_browse_bag)
        self.btnPlayBag.clicked.connect(self.on_play_bag)
        self.btnPauseBag.clicked.connect(self.on_pause_bag)
        self.btnStopBag.clicked.connect(self.on_stop_bag)

        # Connect buttons - SLAM Toolbox tab
        self.btnStartSlamToolboxMapping.clicked.connect(self.on_start_slamtoolbox_mapping)
        self.btnStopSlamToolboxMapping.clicked.connect(self.on_stop_slamtoolbox_mapping)
        self.btnSaveSlamToolboxMap.clicked.connect(self.on_save_slamtoolbox_map)
        self.btnBrowseSlamToolboxMap.clicked.connect(self.on_browse_slamtoolbox_map)
        self.btnStartSlamToolboxLoc.clicked.connect(self.on_start_slamtoolbox_loc)
        self.btnStopSlamToolboxLoc.clicked.connect(self.on_stop_slamtoolbox_loc)
        self.btnRefreshSlamToolboxTopics.clicked.connect(self.on_refresh_slamtoolbox_topics)

        # Connect buttons - Cartographer tab
        self.btnStartCartoMapping.clicked.connect(self.on_start_carto_mapping)
        self.btnStopCartoMapping.clicked.connect(self.on_stop_carto_mapping)
        self.btnSaveCartoMap.clicked.connect(self.on_save_carto_map)
        self.btnBrowseCartoMap.clicked.connect(self.on_browse_carto_map)
        self.btnStartCartoLoc.clicked.connect(self.on_start_carto_loc)
        self.btnStopCartoLoc.clicked.connect(self.on_stop_carto_loc)
        self.btnRefreshCartoTopics.clicked.connect(self.on_refresh_carto_topics)

        # Connect buttons - K-SLAM tab
        self.btnStartKSlam.clicked.connect(self.on_start_kslam)
        self.btnStopKSlam.clicked.connect(self.on_stop_kslam)
        self.btnSaveKSlamMap.clicked.connect(self.on_save_kslam_map)
        self.btnRefreshKSlamTopics.clicked.connect(self.on_refresh_kslam_topics)

        # Connect buttons - RTAB-Map 2D tab
        self.btnStartRtabmap2DMapping.clicked.connect(self.on_start_rtabmap2d_mapping)
        self.btnStopRtabmap2DMapping.clicked.connect(self.on_stop_rtabmap2d_mapping)
        self.btnSaveRtabmap2DMap.clicked.connect(self.on_save_rtabmap2d_map)
        self.btnBrowseRtabmap2DMap.clicked.connect(self.on_browse_rtabmap2d_map)
        self.btnStartRtabmap2DLoc.clicked.connect(self.on_start_rtabmap2d_loc)
        self.btnStopRtabmap2DLoc.clicked.connect(self.on_stop_rtabmap2d_loc)
        self.btnRefreshRtabmap2DTopics.clicked.connect(self.on_refresh_rtabmap2d_topics)

        # Connect buttons - RTAB-Map 2D Camera tab
        self.btnStartRtabmap2DCameraMapping.clicked.connect(self.on_start_rtabmap2d_camera_mapping)
        self.btnStopRtabmap2DCameraMapping.clicked.connect(self.on_stop_rtabmap2d_camera_mapping)
        self.btnSaveRtabmap2DCameraMap.clicked.connect(self.on_save_rtabmap2d_camera_map)
        self.btnBrowseRtabmap2DCameraMap.clicked.connect(self.on_browse_rtabmap2d_camera_map)
        self.btnStartRtabmap2DCameraLoc.clicked.connect(self.on_start_rtabmap2d_camera_loc)
        self.btnStopRtabmap2DCameraLoc.clicked.connect(self.on_stop_rtabmap2d_camera_loc)
        self.btnRefreshRtabmap2DCameraTopics.clicked.connect(self.on_refresh_rtabmap2d_camera_topics)

        # Connect common buttons
        self.btnStopAll.clicked.connect(self.on_stop_all)

        # Status timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)

        # Set default map paths (using workspace_path)
        self.txtSlamToolboxMapPath.setText(
            str(self.workspace_path / "maps" / "slam_toolbox") + "/")
        self.txtCartoMapPath.setText(
            str(self.workspace_path / "maps" / "cartographer") + "/")
        self.txtRtabmap2DMapPath.setText(
            str(self.workspace_path / "maps" / "rtabmap_2d") + "/")
        self.txtRtabmap2DCameraMapPath.setText(
            str(self.workspace_path / "maps" / "rtabmap_2d") + "/")

        self.log("SLAM Manager 2D UI Ready")

    def set_node(self, node):
        """Set the ROS2 node"""
        self.node = node
        self.auto_detect_launch_files()
        self.update_button_states()

    def _is_package_available(self, package_name):
        """Check if ROS2 package is installed"""
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'prefix', package_name],
                capture_output=True, text=True, timeout=5
            )
            return result.returncode == 0
        except Exception:
            return False

    def auto_detect_launch_files(self):
        """Auto-detect launch files by checking installed ROS2 packages"""
        # Package and launch file mapping
        launch_config = {
            'livox': ('livox_ros_driver2', 'msg_MID360_launch.py'),
            'slamtoolbox_mapping': ('slambox_config', 'slam_toolbox_mapping.launch.py'),
            'slamtoolbox_loc': ('slambox_config', 'slam_toolbox_localization.launch.py'),
            'carto_mapping': ('cartographer_slam', 'cartographer.launch.py'),
            'carto_loc': ('cartographer_slam', 'cartographer_localization.launch.py'),
            'kslam': ('k_mapping', 'k_slam.launch.py'),
            'rtabmap2d_mapping': ('rtab_map_config', 'rtabmap_lidar.launch.py'),
            'rtabmap2d_loc': ('rtab_map_config', 'rtabmap_lidar_localization.launch.py'),
            'rtabmap2d_camera_mapping': ('rtab_map_config', 'rtabmap_lidar_camera.launch.py'),
            'rtabmap2d_camera_loc': ('rtab_map_config', 'rtabmap_lidar_camera_localization.launch.py'),
        }

        # Check each package and register launch files
        checked_packages = set()
        for key, (package_name, launch_file) in launch_config.items():
            if package_name not in checked_packages:
                if self._is_package_available(package_name):
                    self.log(f"Found package: {package_name}")
                    checked_packages.add(package_name)
                else:
                    self.log(f"Package not found: {package_name}")

            if package_name in checked_packages:
                self.node.launch_files[key] = (package_name, launch_file)
                self.log(f"  - Registered: {key} -> {launch_file}")

        # Initialize default topic values
        self._init_default_topics()

    def _init_default_topics(self):
        """Initialize default topic values for all comboboxes"""
        default_lidar_topics = ['/scan', '/livox/scan', '/velodyne_points', '/points']
        default_imu_topics = ['/imu/data', '/imu', '/livox/imu']

        # Set default items for all topic comboboxes
        comboboxes = [
            (self.cmbSlamToolboxLidarTopic, self.cmbSlamToolboxImuTopic),
            (self.cmbCartoLidarTopic, self.cmbCartoImuTopic),
            (self.cmbRtabmap2DLidarTopic, self.cmbRtabmap2DImuTopic),
            (self.cmbRtabmap2DCameraLidarTopic, self.cmbRtabmap2DCameraImuTopic),
        ]

        for lidar_cmb, imu_cmb in comboboxes:
            lidar_cmb.clear()
            lidar_cmb.addItems(default_lidar_topics)
            lidar_cmb.setCurrentText('/scan')

            imu_cmb.clear()
            imu_cmb.addItems(default_imu_topics)
            imu_cmb.setCurrentText('/imu/data')

        # K-SLAM (LiDAR only, no IMU)
        self.cmbKSlamLidarTopic.clear()
        self.cmbKSlamLidarTopic.addItems(default_lidar_topics)
        self.cmbKSlamLidarTopic.setCurrentText('/scan')

    def _get_ros2_topics(self):
        """Get list of available ROS2 topics using ros2 topic list"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                topics = [line.strip() for line in result.stdout.split('\n') if line.strip()]
                return topics
            else:
                self.log(f"Failed to get topics: {result.stderr}")
                return []
        except subprocess.TimeoutExpired:
            self.log("ros2 topic list timed out")
            return []
        except Exception as e:
            self.log(f"Error getting topics: {e}")
            return []

    def _filter_lidar_topics(self, topics):
        """Filter topics that are likely LiDAR topics"""
        lidar_keywords = ['scan', 'pointcloud', 'points', 'livox', 'velodyne', 'lidar', 'laser']
        filtered = []
        for topic in topics:
            topic_lower = topic.lower()
            for keyword in lidar_keywords:
                if keyword in topic_lower:
                    filtered.append(topic)
                    break
        return filtered

    def _filter_imu_topics(self, topics):
        """Filter topics that are likely IMU topics"""
        imu_keywords = ['imu', 'orientation', 'angular_velocity', 'linear_acceleration']
        filtered = []
        for topic in topics:
            topic_lower = topic.lower()
            for keyword in imu_keywords:
                if keyword in topic_lower:
                    filtered.append(topic)
                    break
        return filtered

    def _refresh_topic_comboboxes(self, lidar_cmb, imu_cmb):
        """Refresh topic comboboxes with current ROS2 topics"""
        # Save current selections
        current_lidar = lidar_cmb.currentText()
        current_imu = imu_cmb.currentText()

        # Get all topics
        all_topics = self._get_ros2_topics()

        if not all_topics:
            self.log("No topics found. Is ROS2 running?")
            return

        # Filter topics
        lidar_topics = self._filter_lidar_topics(all_topics)
        imu_topics = self._filter_imu_topics(all_topics)

        # Update LiDAR combobox
        lidar_cmb.clear()
        if lidar_topics:
            lidar_cmb.addItems(lidar_topics)
        else:
            # Add defaults if no LiDAR topics found
            lidar_cmb.addItems(['/scan', '/livox/scan'])

        # Restore or set selection
        if current_lidar and lidar_cmb.findText(current_lidar) >= 0:
            lidar_cmb.setCurrentText(current_lidar)
        elif lidar_topics:
            lidar_cmb.setCurrentIndex(0)

        # Update IMU combobox
        imu_cmb.clear()
        if imu_topics:
            imu_cmb.addItems(imu_topics)
        else:
            # Add defaults if no IMU topics found
            imu_cmb.addItems(['/imu/data', '/imu'])

        # Restore or set selection
        if current_imu and imu_cmb.findText(current_imu) >= 0:
            imu_cmb.setCurrentText(current_imu)
        elif imu_topics:
            imu_cmb.setCurrentIndex(0)

        self.log(f"Found {len(lidar_topics)} LiDAR topics, {len(imu_topics)} IMU topics")

    def on_refresh_slamtoolbox_topics(self):
        """Refresh SLAM Toolbox topic comboboxes"""
        self.log("Refreshing SLAM Toolbox topics...")
        self._refresh_topic_comboboxes(
            self.cmbSlamToolboxLidarTopic,
            self.cmbSlamToolboxImuTopic
        )

    def on_refresh_carto_topics(self):
        """Refresh Cartographer topic comboboxes"""
        self.log("Refreshing Cartographer topics...")
        self._refresh_topic_comboboxes(
            self.cmbCartoLidarTopic,
            self.cmbCartoImuTopic
        )

    def on_refresh_rtabmap2d_topics(self):
        """Refresh RTAB-Map 2D topic comboboxes"""
        self.log("Refreshing RTAB-Map 2D topics...")
        self._refresh_topic_comboboxes(
            self.cmbRtabmap2DLidarTopic,
            self.cmbRtabmap2DImuTopic
        )

    def on_refresh_rtabmap2d_camera_topics(self):
        """Refresh RTAB-Map 2D Camera topic comboboxes"""
        self.log("Refreshing RTAB-Map 2D Camera topics...")
        self._refresh_topic_comboboxes(
            self.cmbRtabmap2DCameraLidarTopic,
            self.cmbRtabmap2DCameraImuTopic
        )

    def on_refresh_kslam_topics(self):
        """Refresh K-SLAM topic comboboxes (LiDAR only)"""
        self.log("Refreshing K-SLAM topics...")
        all_topics = self._get_ros2_topics()
        if not all_topics:
            self.log("No topics found. Is ROS2 running?")
            return

        current_lidar = self.cmbKSlamLidarTopic.currentText()
        lidar_topics = self._filter_lidar_topics(all_topics)

        self.cmbKSlamLidarTopic.clear()
        if lidar_topics:
            self.cmbKSlamLidarTopic.addItems(lidar_topics)
        else:
            self.cmbKSlamLidarTopic.addItems(['/scan', '/livox/scan'])

        if current_lidar and self.cmbKSlamLidarTopic.findText(current_lidar) >= 0:
            self.cmbKSlamLidarTopic.setCurrentText(current_lidar)
        elif lidar_topics:
            self.cmbKSlamLidarTopic.setCurrentIndex(0)

        self.log(f"Found {len(lidar_topics)} LiDAR topics")

    def log(self, message):
        """Add message to log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.txtLog.append(f"[{timestamp}] {message}")

    def update_slamtoolbox_position(self, x, y, yaw):
        """Update SLAM Toolbox position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblSlamToolboxPosX.setText(f"X: {x:.3f}")
        self.lblSlamToolboxPosY.setText(f"Y: {y:.3f}")
        self.lblSlamToolboxPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    def update_carto_position(self, x, y, yaw):
        """Update Cartographer position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblCartoPosX.setText(f"X: {x:.3f}")
        self.lblCartoPosY.setText(f"Y: {y:.3f}")
        self.lblCartoPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== Data Source Tab ====================

    def on_start_livox(self):
        launch_info = self.node.launch_files.get('livox')
        if launch_info:
            pkg, launch = launch_info
            if self.node.start_launch_file('livox', pkg, launch):
                self.chkUseSimTime.setChecked(False)
                self.update_button_states()
        else:
            self.log("Livox launch file not configured!")
            QMessageBox.warning(self, "Error", "Livox launch file not found!")

    def on_stop_livox(self):
        if self.node.stop_launch_file('livox'):
            self.update_button_states()

    def on_browse_bag(self):
        """Browse for ROS bag file"""
        default_path = self.workspace_path / 'bag'
        if not default_path.exists():
            default_path = Path.home()

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select ROS Bag",
            str(default_path),
            "ROS Bag (*.db3 *.mcap);;All Files (*)"
        )
        if file_path:
            if file_path.endswith('.db3') or file_path.endswith('.mcap'):
                bag_dir = str(Path(file_path).parent)
                self.txtBagFile.setText(bag_dir)
            else:
                self.txtBagFile.setText(file_path)

    def _get_bag_info(self, bag_path):
        """Get bag duration using ros2 bag info"""
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Duration:' in line:
                        duration_str = line.split('Duration:')[1].strip()
                        total_seconds = 0.0
                        if 'm' in duration_str:
                            parts = duration_str.replace('s', '').split('m')
                            total_seconds = float(parts[0]) * 60 + float(parts[1].strip())
                        else:
                            total_seconds = float(duration_str.replace('s', ''))
                        return total_seconds
        except Exception as e:
            self.log(f"Failed to get bag info: {e}")
        return 0.0

    def on_play_bag(self):
        """Play ROS bag"""
        bag_path = self.txtBagFile.text()
        if not bag_path:
            self.log("Please select a ROS bag file!")
            QMessageBox.warning(self, "Error", "Please select a ROS bag file!")
            return

        if not os.path.exists(bag_path):
            self.log(f"Bag path not found: {bag_path}")
            QMessageBox.warning(self, "Error", f"Bag path not found:\n{bag_path}")
            return

        try:
            self.bag_duration = self._get_bag_info(bag_path)
            self.log(f"Bag duration: {self.bag_duration:.1f}s")

            cmd = ['ros2', 'bag', 'play', bag_path]

            if self.chkBagLoop.isChecked():
                cmd.append('--loop')

            rate = self.spinBagRate.value()
            if rate != 1.0:
                cmd.extend(['--rate', str(rate)])

            if self.chkBagClock.isChecked():
                cmd.append('--clock')
                self.node.set_use_sim_time(True)

            env = os.environ.copy()

            self.bag_process = subprocess.Popen(
                cmd,
                env=env,
                stdin=subprocess.PIPE,
                preexec_fn=os.setpgrp
            )

            self.bag_paused = False
            self.bag_start_time = QDateTime.currentDateTime()
            self.progressBag.setValue(0)
            self.chkUseSimTime.setChecked(True)
            self.log(f"Playing bag: {bag_path}")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to play bag: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to play bag:\n{str(e)}")

    def on_pause_bag(self):
        """Pause/Resume ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.bag_paused = False
                self.log("Bag playback resumed")
            else:
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.bag_paused = True
                self.log("Bag playback paused")

            self._update_bag_button_states()

        except Exception as e:
            self.log(f"Failed to pause/resume bag: {str(e)}")

    def on_stop_bag(self):
        """Stop ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)

            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            time.sleep(1)

            if self.bag_process.poll() is None:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)

            self.bag_process = None
            self.bag_paused = False
            self.bag_start_time = None
            self.node.set_use_sim_time(False)
            self.log("Bag playback stopped")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to stop bag: {str(e)}")
            self.bag_process = None
            self.bag_paused = False
            self.node.set_use_sim_time(False)
            self.bag_start_time = None
            self._update_bag_button_states()

    # ==================== SLAM Toolbox Tab ====================

    def on_start_slamtoolbox_mapping(self):
        self.log("Start Mapping button clicked!")
        launch_info = self.node.launch_files.get('slamtoolbox_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            # Add selected scan topic
            lidar_topic = self.cmbSlamToolboxLidarTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")

            if self.node.start_launch_file('slamtoolbox_mapping', pkg, launch,
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("SLAM Toolbox mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox mapping launch file not found!")

    def on_stop_slamtoolbox_mapping(self):
        if self.node.stop_launch_file('slamtoolbox_mapping'):
            self.update_button_states()

    def on_save_slamtoolbox_map(self):
        """Save SLAM Toolbox map"""
        success, result = self.node.save_slamtoolbox_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_slamtoolbox_map(self):
        """Browse for SLAM Toolbox map file"""
        default_path = str(self.workspace_path / "maps" / "slam_toolbox")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select SLAM Toolbox Map (.posegraph)",
            default_path,
            "Posegraph Files (*.posegraph);;All Files (*)"
        )
        if file_path:
            self.txtSlamToolboxMapPath.setText(file_path)
            self.log(f"SLAM Toolbox map selected: {file_path}")

    def on_start_slamtoolbox_loc(self):
        launch_info = self.node.launch_files.get('slamtoolbox_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtSlamToolboxMapPath.text()

            # Check if path is a .posegraph file
            if not map_path.endswith('.posegraph'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .posegraph file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .posegraph file.")
                return

            # SLAM Toolbox requires map_file_name WITHOUT extension
            map_path_no_ext = map_path[:-10]  # Remove '.posegraph'

            # Also check if .data file exists
            data_file = map_path_no_ext + '.data'
            if not os.path.exists(data_file):
                self.log(f"Map data file not found: {data_file}")
                QMessageBox.warning(self, "Error",
                                    f"Map data file not found:\n{data_file}\n\n"
                                    f"Both .posegraph and .data files are required.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'map_file:={map_path_no_ext}')

            # Add selected topics
            lidar_topic = self.cmbSlamToolboxLidarTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")

            if self.node.start_launch_file('slamtoolbox_loc', pkg, launch, extra_args):
                self.log(f"SLAM Toolbox Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("SLAM Toolbox localization launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox localization launch file not found!")

    def on_stop_slamtoolbox_loc(self):
        if self.node.stop_launch_file('slamtoolbox_loc'):
            self.update_button_states()

    # ==================== Cartographer Tab ====================

    def on_start_carto_mapping(self):
        launch_info = self.node.launch_files.get('carto_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')

            # Add selected topics
            lidar_topic = self.cmbCartoLidarTopic.currentText()
            imu_topic = self.cmbCartoImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('carto_mapping', pkg, launch,
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("Cartographer mapping launch file not configured!")
            QMessageBox.warning(self, "Error",
                                "Cartographer mapping launch file not found!\n\n"
                                "Please configure Cartographer first.")

    def on_stop_carto_mapping(self):
        if self.node.stop_launch_file('carto_mapping'):
            self.update_button_states()

    def on_save_carto_map(self):
        """Save Cartographer map"""
        success, result = self.node.save_carto_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Info", f"Cartographer map save:\n{result}")

    def on_browse_carto_map(self):
        """Browse for Cartographer map file"""
        default_path = str(self.workspace_path / "maps" / "cartographer")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Cartographer Map (.pbstream)",
            default_path,
            "PBStream Files (*.pbstream);;All Files (*)"
        )
        if file_path:
            self.txtCartoMapPath.setText(file_path)
            self.log(f"Cartographer map selected: {file_path}")

    def on_start_carto_loc(self):
        launch_info = self.node.launch_files.get('carto_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtCartoMapPath.text()
            if not map_path or not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .pbstream file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            extra_args.append(f'load_state_filename:={map_path}')

            # Add selected topics
            lidar_topic = self.cmbCartoLidarTopic.currentText()
            imu_topic = self.cmbCartoImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('carto_loc', pkg, launch, extra_args):
                self.log(f"Cartographer Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("Cartographer localization launch file not configured!")
            QMessageBox.warning(self, "Error",
                                "Cartographer localization launch file not found!\n\n"
                                "Please configure Cartographer first.")

    def on_stop_carto_loc(self):
        if self.node.stop_launch_file('carto_loc'):
            self.update_button_states()

    # ==================== K-SLAM Tab ====================

    def on_start_kslam(self):
        launch_info = self.node.launch_files.get('kslam')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            # Add selected scan topic
            lidar_topic = self.cmbKSlamLidarTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")

            if self.node.start_launch_file('kslam', pkg, launch,
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("K-SLAM launch file not configured!")
            QMessageBox.warning(self, "Error", "K-SLAM launch file not found!")

    def on_stop_kslam(self):
        if self.node.stop_launch_file('kslam'):
            self.update_button_states()

    def on_save_kslam_map(self):
        """Save K-SLAM map"""
        success, result = self.node.save_kslam_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def update_kslam_position(self, x, y, yaw):
        """Update K-SLAM position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblKSlamPosX.setText(f"X: {x:.3f}")
        self.lblKSlamPosY.setText(f"Y: {y:.3f}")
        self.lblKSlamPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== RTAB-Map 2D Tab ====================

    def on_start_rtabmap2d_mapping(self):
        launch_info = self.node.launch_files.get('rtabmap2d_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            # Add selected topics
            lidar_topic = self.cmbRtabmap2DLidarTopic.currentText()
            imu_topic = self.cmbRtabmap2DImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('rtabmap2d_mapping', pkg, launch,
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D mapping launch file not found!")

    def on_stop_rtabmap2d_mapping(self):
        if self.node.stop_launch_file('rtabmap2d_mapping'):
            self.update_button_states()

    def on_save_rtabmap2d_map(self):
        """Save RTAB-Map 2D map"""
        success, result = self.node.save_rtabmap2d_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_rtabmap2d_map(self):
        """Browse for RTAB-Map 2D map file"""
        default_path = str(self.workspace_path / "maps" / "rtabmap_2d")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRtabmap2DMapPath.setText(file_path)
            self.log(f"RTAB-Map 2D map selected: {file_path}")

    def on_start_rtabmap2d_loc(self):
        launch_info = self.node.launch_files.get('rtabmap2d_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtRtabmap2DMapPath.text()

            # Check if path is a .db file
            if not map_path.endswith('.db'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .db file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .db file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'database_path:={map_path}')

            # Add selected topics
            lidar_topic = self.cmbRtabmap2DLidarTopic.currentText()
            imu_topic = self.cmbRtabmap2DImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('rtabmap2d_loc', pkg, launch, extra_args):
                self.log(f"RTAB-Map 2D Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D localization launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D localization launch file not found!")

    def on_stop_rtabmap2d_loc(self):
        if self.node.stop_launch_file('rtabmap2d_loc'):
            self.update_button_states()

    def update_rtabmap2d_position(self, x, y, yaw):
        """Update RTAB-Map 2D position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblRtabmap2DPosX.setText(f"X: {x:.3f}")
        self.lblRtabmap2DPosY.setText(f"Y: {y:.3f}")
        self.lblRtabmap2DPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== RTAB-Map 2D Camera Tab ====================

    def on_start_rtabmap2d_camera_mapping(self):
        launch_info = self.node.launch_files.get('rtabmap2d_camera_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            # Add selected topics
            lidar_topic = self.cmbRtabmap2DCameraLidarTopic.currentText()
            imu_topic = self.cmbRtabmap2DCameraImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('rtabmap2d_camera_mapping', pkg, launch,
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D Camera mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D Camera mapping launch file not found!")

    def on_stop_rtabmap2d_camera_mapping(self):
        if self.node.stop_launch_file('rtabmap2d_camera_mapping'):
            self.update_button_states()

    def on_save_rtabmap2d_camera_map(self):
        """Save RTAB-Map 2D Camera map"""
        success, result = self.node.save_rtabmap2d_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_rtabmap2d_camera_map(self):
        """Browse for RTAB-Map 2D Camera map file"""
        default_path = str(self.workspace_path / "maps" / "rtabmap_2d")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRtabmap2DCameraMapPath.setText(file_path)
            self.log(f"RTAB-Map 2D Camera map selected: {file_path}")

    def on_start_rtabmap2d_camera_loc(self):
        launch_info = self.node.launch_files.get('rtabmap2d_camera_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtRtabmap2DCameraMapPath.text()

            # Check if path is a .db file
            if not map_path.endswith('.db'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .db file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .db file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'database_path:={map_path}')

            # Add selected topics
            lidar_topic = self.cmbRtabmap2DCameraLidarTopic.currentText()
            imu_topic = self.cmbRtabmap2DCameraImuTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")
            if imu_topic:
                extra_args.append(f'imu_topic:={imu_topic}')
                self.log(f"Using IMU topic: {imu_topic}")

            if self.node.start_launch_file('rtabmap2d_camera_loc', pkg, launch, extra_args):
                self.log(f"RTAB-Map 2D Camera Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D Camera localization launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D Camera localization launch file not found!")

    def on_stop_rtabmap2d_camera_loc(self):
        if self.node.stop_launch_file('rtabmap2d_camera_loc'):
            self.update_button_states()

    def update_rtabmap2d_camera_position(self, x, y, yaw):
        """Update RTAB-Map 2D Camera position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblRtabmap2DCameraPosX.setText(f"X: {x:.3f}")
        self.lblRtabmap2DCameraPosY.setText(f"Y: {y:.3f}")
        self.lblRtabmap2DCameraPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== Common ====================

    def on_stop_all(self):
        reply = QMessageBox.question(
            self,
            "Confirm",
            "Stop all running processes?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.node.stop_all_launches()
            if self.bag_process is not None:
                self.on_stop_bag()
            self.update_button_states()

    def _format_time(self, seconds):
        """Format seconds to MM:SS"""
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes:02d}:{secs:02d}"

    def _update_bag_button_states(self):
        """Update ROS bag button states"""
        bag_running = self.bag_process is not None and self.bag_process.poll() is None

        self.btnPlayBag.setEnabled(not bag_running)
        self.btnPauseBag.setEnabled(bag_running)
        self.btnStopBag.setEnabled(bag_running)

        if self.bag_paused:
            self.btnPauseBag.setText("Resume")
        else:
            self.btnPauseBag.setText("Pause")

        if bag_running and self.bag_duration > 0 and self.bag_start_time is not None:
            elapsed_ms = self.bag_start_time.msecsTo(QDateTime.currentDateTime())
            rate = self.spinBagRate.value()
            elapsed_sec = (elapsed_ms / 1000.0) * rate

            if self.chkBagLoop.isChecked():
                elapsed_sec = elapsed_sec % self.bag_duration

            progress = min(100, int((elapsed_sec / self.bag_duration) * 100))
            self.progressBag.setValue(progress)

            current_time = self._format_time(min(elapsed_sec, self.bag_duration))
            total_time = self._format_time(self.bag_duration)
            self.progressBag.setFormat(f"{progress}% - {current_time} / {total_time}")
        elif not bag_running:
            if self.bag_process is None:
                self.progressBag.setValue(0)
                self.progressBag.setFormat("0% - 00:00 / 00:00")

    def update_button_states(self):
        """Update all button states based on running processes"""
        if self.node is None:
            return

        # Style definitions
        style_ready = "background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;"
        style_running = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_loc = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_save = "background-color: #FF9800; color: white; font-weight: bold; padding: 10px;"

        # Check running states
        livox_running = self.node.is_running('livox')
        bag_running = self.bag_process is not None and self.bag_process.poll() is None
        data_source_available = livox_running or bag_running

        slamtoolbox_mapping_running = self.node.is_running('slamtoolbox_mapping')
        slamtoolbox_loc_running = self.node.is_running('slamtoolbox_loc')
        carto_mapping_running = self.node.is_running('carto_mapping')
        carto_loc_running = self.node.is_running('carto_loc')

        # Livox
        self.btnStartLivox.setEnabled(not livox_running)
        self.btnStopLivox.setEnabled(livox_running)
        if livox_running:
            self.btnStartLivox.setStyleSheet(style_running)
            self.lblLivoxStatus.setText("Status: Running")
        else:
            self.btnStartLivox.setStyleSheet(style_ready)
            self.lblLivoxStatus.setText("Status: Stopped")

        # SLAM Toolbox Mapping (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        self.btnStartSlamToolboxMapping.setEnabled(
            not slamtoolbox_mapping_running and not slamtoolbox_loc_running)
        self.btnStopSlamToolboxMapping.setEnabled(slamtoolbox_mapping_running)
        self.btnSaveSlamToolboxMap.setEnabled(slamtoolbox_mapping_running)
        if slamtoolbox_mapping_running:
            self.btnStartSlamToolboxMapping.setStyleSheet(style_running)
        else:
            self.btnStartSlamToolboxMapping.setStyleSheet(style_ready)
        self.btnSaveSlamToolboxMap.setStyleSheet(style_save)

        # SLAM Toolbox Localization (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        self.btnStartSlamToolboxLoc.setEnabled(
            not slamtoolbox_loc_running and not slamtoolbox_mapping_running)
        self.btnStopSlamToolboxLoc.setEnabled(slamtoolbox_loc_running)
        if slamtoolbox_loc_running:
            self.btnStartSlamToolboxLoc.setStyleSheet(style_running)
        else:
            self.btnStartSlamToolboxLoc.setStyleSheet(style_loc)

        # Cartographer Mapping (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        carto_configured = self.node.launch_files['carto_mapping'] is not None
        self.btnStartCartoMapping.setEnabled(
            carto_configured and not carto_mapping_running and not carto_loc_running)
        self.btnStopCartoMapping.setEnabled(carto_mapping_running)
        self.btnSaveCartoMap.setEnabled(carto_mapping_running)
        if carto_mapping_running:
            self.btnStartCartoMapping.setStyleSheet(style_running)
        else:
            self.btnStartCartoMapping.setStyleSheet(style_ready)
        self.btnSaveCartoMap.setStyleSheet(style_save)

        # Cartographer Localization (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        carto_loc_configured = self.node.launch_files['carto_loc'] is not None
        self.btnStartCartoLoc.setEnabled(
            carto_loc_configured and not carto_loc_running and not carto_mapping_running)
        self.btnStopCartoLoc.setEnabled(carto_loc_running)
        if carto_loc_running:
            self.btnStartCartoLoc.setStyleSheet(style_running)
        else:
            self.btnStartCartoLoc.setStyleSheet(style_loc)

        # Update Cartographer not configured label
        if not carto_configured:
            self.lblCartoNotConfigured.setVisible(True)
        else:
            self.lblCartoNotConfigured.setVisible(False)

        # K-SLAM
        kslam_running = self.node.is_running('kslam')
        kslam_configured = self.node.launch_files['kslam'] is not None
        self.btnStartKSlam.setEnabled(kslam_configured and not kslam_running)
        self.btnStopKSlam.setEnabled(kslam_running)
        self.btnSaveKSlamMap.setEnabled(kslam_running)
        if kslam_running:
            self.btnStartKSlam.setStyleSheet(style_running)
        else:
            self.btnStartKSlam.setStyleSheet(style_ready)
        self.btnSaveKSlamMap.setStyleSheet(style_save)

        # RTAB-Map 2D
        rtabmap2d_mapping_running = self.node.is_running('rtabmap2d_mapping')
        rtabmap2d_loc_running = self.node.is_running('rtabmap2d_loc')
        rtabmap2d_mapping_configured = self.node.launch_files['rtabmap2d_mapping'] is not None
        rtabmap2d_loc_configured = self.node.launch_files['rtabmap2d_loc'] is not None

        # RTAB-Map 2D Mapping
        self.btnStartRtabmap2DMapping.setEnabled(
            rtabmap2d_mapping_configured and not rtabmap2d_mapping_running and not rtabmap2d_loc_running)
        self.btnStopRtabmap2DMapping.setEnabled(rtabmap2d_mapping_running)
        self.btnSaveRtabmap2DMap.setEnabled(rtabmap2d_mapping_running)
        if rtabmap2d_mapping_running:
            self.btnStartRtabmap2DMapping.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DMapping.setStyleSheet(style_ready)
        self.btnSaveRtabmap2DMap.setStyleSheet(style_save)

        # RTAB-Map 2D Localization
        self.btnStartRtabmap2DLoc.setEnabled(
            rtabmap2d_loc_configured and not rtabmap2d_loc_running and not rtabmap2d_mapping_running)
        self.btnStopRtabmap2DLoc.setEnabled(rtabmap2d_loc_running)
        if rtabmap2d_loc_running:
            self.btnStartRtabmap2DLoc.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DLoc.setStyleSheet(style_loc)

        # RTAB-Map 2D Camera
        rtabmap2d_camera_mapping_running = self.node.is_running('rtabmap2d_camera_mapping')
        rtabmap2d_camera_loc_running = self.node.is_running('rtabmap2d_camera_loc')
        rtabmap2d_camera_mapping_configured = self.node.launch_files.get('rtabmap2d_camera_mapping') is not None
        rtabmap2d_camera_loc_configured = self.node.launch_files.get('rtabmap2d_camera_loc') is not None

        # RTAB-Map 2D Camera Mapping
        self.btnStartRtabmap2DCameraMapping.setEnabled(
            rtabmap2d_camera_mapping_configured and not rtabmap2d_camera_mapping_running and not rtabmap2d_camera_loc_running)
        self.btnStopRtabmap2DCameraMapping.setEnabled(rtabmap2d_camera_mapping_running)
        self.btnSaveRtabmap2DCameraMap.setEnabled(rtabmap2d_camera_mapping_running)
        if rtabmap2d_camera_mapping_running:
            self.btnStartRtabmap2DCameraMapping.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DCameraMapping.setStyleSheet(style_ready)
        self.btnSaveRtabmap2DCameraMap.setStyleSheet(style_save)

        # RTAB-Map 2D Camera Localization
        self.btnStartRtabmap2DCameraLoc.setEnabled(
            rtabmap2d_camera_loc_configured and not rtabmap2d_camera_loc_running and not rtabmap2d_camera_mapping_running)
        self.btnStopRtabmap2DCameraLoc.setEnabled(rtabmap2d_camera_loc_running)
        if rtabmap2d_camera_loc_running:
            self.btnStartRtabmap2DCameraLoc.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DCameraLoc.setStyleSheet(style_loc)

        # ROS Bag
        self._update_bag_button_states()

    def closeEvent(self, event):
        """Handle window close event"""
        reply = QMessageBox.question(
            self,
            "Confirm Exit",
            "Stop all processes and exit?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.node:
                self.node.stop_all_launches()
            if self.bag_process is not None:
                self.on_stop_bag()
            event.accept()
        else:
            event.ignore()
