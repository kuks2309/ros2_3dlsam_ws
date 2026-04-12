# RouteNode - Graph node representing a waypoint in the ODD
uint32 id
float64 x           # map frame (m)
float64 y           # map frame (m)
float64 yaw         # (rad)
string drive_mode   # TRANSLATE, BICYCLE, AUTO, etc.
float64 speed       # (m/s)
float64 timeout     # (s)
