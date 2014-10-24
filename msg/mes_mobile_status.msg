Header header

# Version of the client
uint8 version_id

# When the robot has completed a task it shall return STATE_FREE
uint8 STATE_FREE=0
uint8 STATE_ERROR=1
uint8 STATE_WORKING=2
uint8 state

# For future use. Return 0 when not working
uint8 done_pct

# This is the raw value from the frobomind controller taken directly from the ADC
uint16 battery 

# The position of the mobile platform
string position

# Feel free to use this to write a human readable status of the robot. This may show up on the MES dashboard.
string status
