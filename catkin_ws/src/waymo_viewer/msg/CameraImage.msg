### CameraImage.msg

Header header

## Name
#	UNKNOWN = 0;
#   FRONT = 1;
#   FRONT_LEFT = 2;
#   FRONT_RIGHT = 3;
#   SIDE_LEFT = 4;
#   SIDE_RIGHT = 5;

int8 name
float64 shutter
float64 camera_trigger_time
float64 camera_readout_done_time
sensor_msgs/Image image
float64[] intrinsic
float64[] extrinsic

## RollingShutterReadOutDirection
#	UNKNOWN = 0;
#	TOP_TO_BOTTOM = 1;
#	LEFT_TO_RIGHT = 2;
#	BOTTOM_TO_TOP = 3;
#	RIGHT_TO_LEFT = 4;
#	GLOBAL_SHUTTER = 5;

int8 rolling_shutter_direction

Label[] labels
