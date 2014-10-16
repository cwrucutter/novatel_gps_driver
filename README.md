novatel_gps_driver
==================

CWRUCutter's Novatel GPS driver. Uses output from Novatel's propak 3 to publishes the following messages:

Position estimate
type `/sensor_msgs/NavSatFix` to `/gps_fix`

Velocity estimate
type `/geometry_msgs/TwistStamped` to `/gps_vel`

GPS status
type `/sensor_msgs/GPSStatus` to `/gps_status`
