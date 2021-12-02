#include <stdio.h>

#include "Kalman.h"
#include "PID.h"

struct controller_output Controller (struct state state, float roll_des, float pitch_des, float yaw_rate_des, float gyro[3], float acc[3]);
