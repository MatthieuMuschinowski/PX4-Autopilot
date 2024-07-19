uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 windspeed_north		# Raw wind component in north / X direction (m/sec)
float32 windspeed_east		# Raw wind component in east / Y direction (m/sec)
float32 windspeed_down		# Raw wind component in down / Z direction (m/sec)

float32 windspeed_north_f		# Filtered wind component in north / X direction (m/sec)
float32 windspeed_east_f		# Filtered wind component in east / Y direction (m/sec)
float32 windspeed_down_f		# Filtered wind component in down / Z direction (m/sec)

float32 variance_north		# Wind estimate error variance in north / X direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
float32 variance_east		  # Wind estimate error variance in east / Y direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
float32 variance_down		  # Wind estimate error variance in down / Z direction (m/sec)**2 - set to zero (no uncertainty) if not estimated

float32 wind_alt          # Altitude (MSL) that this measurement was taken at (NAN if unknown)
float32 horiz_accuracy    # Horizontal speed 1-STD accuracy (0 if unknown)
float32 vert_accuracy     # Vertical speed 1-STD accuracy (0 if unknown)

bool multiprobe     # Are we in multiprobe mode

# TOPICS wind_measure
