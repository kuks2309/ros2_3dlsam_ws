# waypoint_interfaces/msg/Segment.msg
uint8 SPIN              = 0
uint8 TRANSLATE         = 1
uint8 TRANSLATE_REVERSE = 2
uint8 TURN              = 3
uint8 YAWCTRL           = 4
uint8 WAIT              = 5

# Control mode (Translate 전용, 0=node param default)
uint8 CTRL_DEFAULT  = 0
uint8 CTRL_MODE_A   = 1
uint8 CTRL_MODE_B   = 2
uint8 CTRL_MODE_C   = 3

uint32  segment_id
uint32  waypoint_from         # 출발 Waypoint ID
uint32  waypoint_to           # 도착 Waypoint ID
uint8   action_type           # SPIN / TRANSLATE / TRANSLATE_REVERSE / TURN / YAWCTRL / WAIT
uint8   control_mode 0        # Translate 전용 (CTRL_DEFAULT/A/B/C)

# Spin 파라미터
float64 spin_angle            # deg

# Translate/YawControl 파라미터
float64 start_x
float64 start_y
float64 end_x
float64 end_y
float64 max_linear_speed
float64 acceleration

# Turn 파라미터
float64 turn_radius           # m
float64 turn_angle            # deg

# Wait 파라미터
float64 wait_duration         # sec (WAIT 모드 대기 시간)

# 공통
bool    hold_steer            # 다음 Segment 존재 시 true
float64 exit_steer_angle
float64 exit_speed            # m/s (0=완전정지, >0=이 속도로 종료)
bool    has_next              # 다음 Segment 존재 여부
