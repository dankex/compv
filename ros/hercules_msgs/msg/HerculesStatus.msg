Header header

# MCU Uptime, in ms
uint32 uptime

# ROS Control loop frequency (PC-side)
float64 ros_control_loop_freq

# Voltage of platform components, in volts
float64 battery_voltage

# Hercules error/stop conditions
bool timeout
bool lockout
bool e_stop
bool ros_pause
bool no_battery
bool current_limit
