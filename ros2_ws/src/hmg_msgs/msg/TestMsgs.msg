# Message from external ROS Node to CarMaker ROS Node
std_msgs/Header header
builtin_interfaces/Time time
int64 cycleno

# Control inputs
float64 steering    # 조향각 [-1.0 ~ 1.0] normalized
float64 gas         # 가속   [ 0.0 ~ 1.0]
float64 brake       # 브레이크 [ 0.0 ~ 1.0]
int32 gear
