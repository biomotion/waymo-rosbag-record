### Lidar.msg

Header header

## Name
#	UNKNOWN = 0;
#   TOP = 1;
#   FRONT = 2;
#   SIDE_LEFT = 3;
#   SIDE_RIGHT = 4;
#   REAR = 5;

int8 name
sensor_msgs/PointCloud pointcloud_ri1
float64[] extrinsic
float64[] beam_inclinations
float64 beam_inclination_min
float64 beam_inclination_max

Label[] labels
