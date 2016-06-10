# RobotStatus.msg
# Message format for informing the OCU
# about state of a particular robot

# battery status
int8 battery

# turtle factor
uint8 turtle_factor

# is battery charging
bool battery_charging

# is drive direction reversed
bool drive_reversed

# is emergency stop active
bool emergency_active

# is brake active
bool brake_active

