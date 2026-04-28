"""Pre-launch duplicate-node detection for ROS2 launch files.

ROS2 launch 시작 직전에 동일한 이름(`__node:=NAME`)을 가진 노드가 이미
실행 중인지 검사한다. 중복 발견 시 동작은 launch 인자 ``auto_kill`` 로 결정:

- ``auto_kill:=false`` (기본) — launch 중단(Shutdown) + 진단 메시지 출력
- ``auto_kill:=true``         — 기존 PID 강제 종료(SIGKILL) 후 launch 진행

사용법::

    from launch import LaunchDescription
    from launch_utils import setup_dup_check, setup_gpu_offload

    def generate_launch_description():
        return LaunchDescription([
            *setup_dup_check([
                'rviz2', 'rqt_robot_steering', 'robot_state_publisher',
            ]),
            *setup_gpu_offload(),
            # ... 나머지 launch actions
        ])

검출 방식: ``ps -eo pid,cmd`` 출력에서 ``__node:=NAME`` 패턴(이름 끝이
공백 또는 줄끝)을 정규식으로 매칭. ROS2 launch 시스템이 모든 Node action
의 cmdline에 ``__node:=`` 형태로 노드 이름을 주입하므로 신뢰 가능.
"""

import os
import re
import signal
import subprocess
import time

from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration


def _find_dup_pids(node_name: str):
    """주어진 노드 이름과 일치하는 실행 중 ROS2 노드 PID 목록 반환."""
    try:
        out = subprocess.run(
            ['ps', '-eo', 'pid,cmd'],
            capture_output=True, text=True, timeout=2,
        ).stdout
    except Exception:
        return []
    pat = re.compile(rf'__node:={re.escape(node_name)}(?:\s|$)')
    pids = []
    for line in out.splitlines()[1:]:
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        pid_s, cmd = parts
        if pat.search(cmd):
            try:
                pids.append(int(pid_s))
            except ValueError:
                pass
    return pids


def _make_check(node_names):
    def _check(context):
        auto_kill = (
            LaunchConfiguration('auto_kill').perform(context).lower() == 'true'
        )
        actions = []
        duplicates = {}
        for name in node_names:
            pids = _find_dup_pids(name)
            if pids:
                duplicates[name] = pids

        if not duplicates:
            return [LogInfo(msg='[dup_check] OK — 중복 노드 없음')]

        if auto_kill:
            actions.append(LogInfo(
                msg=f'[dup_check] auto_kill=true — 중복 노드 강제 종료: {duplicates}'
            ))
            for name, pids in duplicates.items():
                for pid in pids:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    except Exception as e:
                        actions.append(LogInfo(
                            msg=f'[dup_check] kill PID={pid} 실패: {e}'
                        ))
            time.sleep(1.0)
            return actions

        # auto_kill=false → 중단
        for name, pids in duplicates.items():
            actions.append(LogInfo(
                msg=f'[dup_check] DUPLICATE: {name} (PIDs={pids})'
            ))
        actions.append(LogInfo(msg=(
            '[dup_check] 중복 노드 발견 — launch 중단. '
            '해결 방법: (1) scripts/kill_all_ros2.sh 실행 후 재시도, '
            '(2) 강제 진행하려면 auto_kill:=true 인자 추가'
        )))
        actions.append(Shutdown(reason='duplicate ROS2 nodes detected'))
        return actions

    return _check


def setup_dup_check(node_names):
    """LaunchDescription에 unpack(*)으로 넣을 액션 리스트 반환.

    Args:
        node_names: 검사할 노드 이름 목록 (Node(name=...) 와 동일하게 지정).

    Returns:
        ``[DeclareLaunchArgument('auto_kill'), OpaqueFunction(...)]`` 두 개.
    """
    return [
        DeclareLaunchArgument(
            'auto_kill',
            default_value='false',
            description=(
                'Auto-kill duplicate ROS2 nodes before launch '
                '(default: abort with diagnostic)'
            ),
        ),
        OpaqueFunction(function=_make_check(list(node_names))),
    ]
