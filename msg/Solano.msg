uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 windspeed		# Raw wind speed measured by the probe (m/sec)
float32 angle_ew		# Raw wind direction in East / West component (°)
float32 angle_ns		# Raw wind direction in North / South component (°)

float32 windspeed_f		# Filtered wind speed measured by the probe (m/sec)
float32 angle_ew_f		# Filtered wind direction in East / West component (°)
float32 angle_ns_f		# Filtered wind direction in North / South component (°)

uint8 id  # Id of the probe : main probe is always 1, if id=0 probe is unregistered

# TOPICS solano
