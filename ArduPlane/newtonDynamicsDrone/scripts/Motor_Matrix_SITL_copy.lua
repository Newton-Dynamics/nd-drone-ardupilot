MAV_SEVERITY_INFO = 6

-- yaw factors (you probably want CW = 1, CCW = -1 for real yaw authority)
local AP_MOTORS_MATRIX_YAW_FACTOR_CW  = -1
local AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1

-- motor indices are 0-based
-- (you can either keep these or just write 0..7 directly)
local M1 = 0
local M2 = 1
local M3 = 2
local M4 = 3
local M5 = 4
local M6 = 5
local M7 = 6
local M8 = 7

AP_MOTORS_MAX_NUM_MOTORS = 8

-- roll, pitch, yaw, testing_order

MotorsMatrix:add_motor_raw(M1, -0.441,  0.817,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1)
MotorsMatrix:add_motor_raw(M2,  0.441, -1.000,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5)
MotorsMatrix:add_motor_raw(M3, -0.441,  0.398,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
MotorsMatrix:add_motor_raw(M4, -0.441, -1.000,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4)
MotorsMatrix:add_motor_raw(M5,  0.441,  0.817,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8)
MotorsMatrix:add_motor_raw(M6,  0.441, -0.215,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6)
MotorsMatrix:add_motor_raw(M7,  0.441,  0.398,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7)
MotorsMatrix:add_motor_raw(M8, -0.441, -0.215,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)

--[[ 
-- disrupted case: out of order additions
MotorsMatrix:add_motor_raw(M1, -0.48, 0.85, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
MotorsMatrix:add_motor_raw(M2, 0.441, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
MotorsMatrix:add_motor_raw(M3, -0.441, 0.44, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
MotorsMatrix:add_motor_raw(M4, -0.441, -0.97, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
MotorsMatrix:add_motor_raw(M5, 0.441, 0.84, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
MotorsMatrix:add_motor_raw(M6, 0.441, -0.22, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
MotorsMatrix:add_motor_raw(M7, 0.441, 0.44, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7);
MotorsMatrix:add_motor_raw(M8, -0.441, -0.215, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8);
--]]

-- initialise the mixer with 8 motors
assert(MotorsMatrix:init(8), "Failed to init the full MotorsMatrix")

gcs:send_text(MAV_SEVERITY_INFO, "LUA: MotorMatrix loaded")

