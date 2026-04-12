std_msgs/Header header
uint8 STATUS_FREE    = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_BLOCKED = 2
uint8 STATUS_UNKNOWN = 3
uint8 status 3
int32   obstacle_points
float64 nearest_distance
float64 nearest_angle
float64 nearest_x
float64 nearest_y
bool    costmap_received
int32   corridor_free_cells
