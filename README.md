novatel_gps_driver
==================

CWRUCutter's Novatel GPS driver. Uses output from Novatel's propak 3 to publishes the following messages:

Position estimate - message type `/sensor_msgs/NavSatFix` topic `/gps_fix`

Velocity estimate - message type `/geometry_msgs/TwistStamped` topic `/gps_vel`

GPS status - message type `/sensor_msgs/GPSStatus` topic `/gps_status`
