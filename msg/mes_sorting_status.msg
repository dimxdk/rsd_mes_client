Header header

# Version of the client
uint8 version_id

# When the platform is done loading, the state should be considered free. The MES frees the resource when the mobile platform has left the cell.
uint8 STATE_FREE=0
uint8 STATE_SORTING=1
uint8 STATE_OUTOFBRICKS=2
uint8 STATE_ORDERSORTED=3
uint8 STATE_LOADING=4
uint8 STATE_ERROR=5

uint8 state

# For future use. Return 0 when not working
uint8 done_pct

# Feel free to use this to write a human readable status of the robot. This may show up on the MES dashboard.
string status

