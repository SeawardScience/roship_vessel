std_msgs/Header header

float64 tension_lbs       # cable tension (pounds)
float64 speed_m_per_min   # pay-in/pay-out speed (m/min, positive = paying out)
float64 payout_m          # total cable paid out (meters)

string  device_timestamp  # ISO 8601 timestamp reported by the device
uint32  checksum          # checksum value from message
bool    checksum_valid     # true if computed checksum matched reported value
