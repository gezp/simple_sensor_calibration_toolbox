# frame id
string frame_id

# accelerometer intrinsic: corrected = matrix * (raw - bias)
float64[9] accel_matrix
float64[3] accel_bias
# white noise density [m/s^2/sqrt(Hz)]
float64[3] accel_noise_density
# bias random walk [m/s^3/sqrt(Hz)]
float64[3] accel_random_walk

# gyroscope intrinsic: corrected = matrix * (raw - bias)
float64[9] gyro_matrix
float64[3] gyro_bias
# white noise density [rad/s/sqrt(Hz)]
float64[3] gyro_noise_density
# bias random walk [rad/s^2/sqrt(Hz)]
float64[3] gyro_random_walk
