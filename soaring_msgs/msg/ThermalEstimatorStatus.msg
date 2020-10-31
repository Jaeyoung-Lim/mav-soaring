# reference for polynomial trajectory tracking
#

std_msgs/Header header

uint16 status
uint16 THERMAL_UNKNOWN = 0
uint16 THERMAL_FIXED = 1

float64 netto_variometer # Netto Variometer
float64 latitude # Thermal Center lattitude
float64 longitude # Thermal Center longitude
float32 altitude # Thermal Center longitude
float64 radius # Thermal Radius
float64 strength # Themral Strength
geometry_msgs/Vector3 position
