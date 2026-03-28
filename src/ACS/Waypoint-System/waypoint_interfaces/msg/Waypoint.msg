# waypoint_interfaces/msg/Waypoint.msg
uint32  id                    # 경유지 고유 번호
float64 x                    # map X (m)
float64 y                    # map Y (m)
float64 heading              # 도착 시 목표 heading (rad, NaN이면 무시)
float64 max_speed            # 이 구간 최대 속도 (m/s, 0이면 기본값 사용)
uint8   arrival_action       # 도착 후 동작 (0=없음, 1=정지 후 대기, 2=데이터수집)
float64 wait_duration        # arrival_action=1일 때 대기 시간 (sec)

# Drive mode: 이 WP까지 도달하는 주행 모드
uint8   DRIVE_AUTO      = 0  # 기존 자동 분해 (Spin + Translate)
uint8   DRIVE_TRANSLATE = 1  # 직진
uint8   DRIVE_TURN      = 3  # 원호 주행
uint8   DRIVE_SPIN      = 4  # 제자리 회전만
uint8   DRIVE_YAWCTRL       = 5  # Yaw 보정 직진
uint8   DRIVE_PURE_STANLEY  = 2  # Pure Stanley 직진
uint8   DRIVE_WAIT          = 6  # 지정 시간 대기 (wait_duration 사용)
uint8   drive_mode 0

# Turn 모드 전용 파라미터
float64 turn_radius 0.0      # m (TURN 모드 시 회전 반경)

# Spin 모드 전용 파라미터
float64 spin_angle 0.0       # deg (SPIN 모드 상대 회전각, +CCW/-CW, 0이면 heading 사용)
