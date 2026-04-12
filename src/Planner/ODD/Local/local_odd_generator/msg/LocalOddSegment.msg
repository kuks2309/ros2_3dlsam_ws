# local_odd_generator/msg/LocalOddSegment.msg
# 경로 내 단일 구간의 ODD 제약

uint32  start_index
uint32  end_index
float64 start_distance 0.0
float64 end_distance   0.0

uint32 source_edge_id  0

float64 speed_limit    0.0
float64 accel_limit    0.0
float64 decel_limit    0.0

float64 max_delta_deg  0.0

uint8 DIRECTION_BOTH         = 0
uint8 DIRECTION_FORWARD_ONLY = 1
uint8 DIRECTION_REVERSE_ONLY = 2
uint8 direction 0

uint8 ZONE_NORMAL = 0
uint8 ZONE_SLOW   = 1
uint8 ZONE_STOP   = 2
uint8 zone_type 0

float64 path_width 0.0

uint8 FLAG_NO_REVERSE = 1
uint8 FLAG_PAUSE      = 2
uint8 FLAG_SLOPE      = 4
uint8 FLAG_SHARED     = 8
uint8 flags 0
