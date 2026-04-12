# RouteEdge - Graph edge connecting two RouteNodes
uint32 id
uint32 from_node_id
uint32 to_node_id
bool bidirectional
uint8 edge_type         # 0=STRAIGHT, 1=ARC, 2=BEZIER, 3=DUBINS
float64 weight          # distance or cost (auto-calculated)
float64 path_width      # (m) corridor width, 0.0 = use default

# Edge type constants
uint8 STRAIGHT=0
uint8 ARC=1
uint8 BEZIER=2
uint8 DUBINS=3
