# local_odd_generator/msg/LocalOddMap.msg
# 경로 전체의 ODD 제약 배열

std_msgs/Header header
uint32 path_poses_count 0
LocalOddSegment[] segments
