-- NewtonTwo - Series 1
-- Updated 11 Feb 2026
-- Author: Hussein Sleiman
-- Custom Motor Matrix Lua for H-Frame with 8 motors

-- MAVLink severity levels for GCS messages
local MAV_SEVERITY_EMERGENCY = 0
local MAV_SEVERITY_NOTICE    = 5
local MAV_SEVERITY_INFO      = 6

-- yaw direction constants (match AP_MotorsMatrix)
local AP_MOTORS_MATRIX_YAW_FACTOR_CW  = -1
local AP_MOTORS_MATRIX_YAW_FACTOR_CCW =  1

----------------------------------------------------------------------
-- MOTOR MATRIX inputs to iforce2d.net
-- https://www.iforce2d.net/mixercalc/
-- 1 3 396
-- 3 8 594
-- 8 4 664
-- 5 7 369
-- 7 6 594
-- 6 2 664
-- 1 3 v
-- 3 8 v
-- 8 4 v
-- 5 7 v
-- 7 6 v
-- 6 2 v
-- 5 1 806
-- 7 3 806
-- 6 8 806
-- 2 4 806
-- 5 1 h
-- 7 3 h
-- 6 8 h
-- 2 4 h
----------------------------------------------------------------------

-- Motor indices (0-based)
local M1 = 0
local M2 = 1
local M3 = 2
local M4 = 3
local M5 = 4
local M6 = 5
local M7 = 6
local M8 = 7

-- Motor geometry (normalised roll/pitch factors)
local NEWTON_MOT_01_X = -0.452
local NEWTON_MOT_01_Y =  0.846

local NEWTON_MOT_02_X =  0.452
local NEWTON_MOT_02_Y = -0.999

local NEWTON_MOT_03_X = -0.452
local NEWTON_MOT_03_Y =  0.410

local NEWTON_MOT_04_X = -0.452
local NEWTON_MOT_04_Y = -1.000

local NEWTON_MOT_05_X =  0.452
local NEWTON_MOT_05_Y =  0.838

local NEWTON_MOT_06_X =  0.452
local NEWTON_MOT_06_Y = -0.254

local NEWTON_MOT_07_X =  0.452
local NEWTON_MOT_07_Y =  0.415

local NEWTON_MOT_08_X = -0.452
local NEWTON_MOT_08_Y = -0.255

----------------------------------------------------------------------
-- Motor matrix initialisation at script load
----------------------------------------------------------------------
MotorsMatrix:add_motor_raw(M1, NEWTON_MOT_01_X, NEWTON_MOT_01_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1)
MotorsMatrix:add_motor_raw(M2, NEWTON_MOT_02_X, NEWTON_MOT_02_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5)
MotorsMatrix:add_motor_raw(M3, NEWTON_MOT_03_X, NEWTON_MOT_03_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
MotorsMatrix:add_motor_raw(M4, NEWTON_MOT_04_X, NEWTON_MOT_04_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4)

--

MotorsMatrix:add_motor_raw(M5, NEWTON_MOT_05_X, NEWTON_MOT_05_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8)
MotorsMatrix:add_motor_raw(M6, NEWTON_MOT_06_X, NEWTON_MOT_06_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6)
MotorsMatrix:add_motor_raw(M7, NEWTON_MOT_07_X, NEWTON_MOT_07_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7)
MotorsMatrix:add_motor_raw(M8, NEWTON_MOT_08_X, NEWTON_MOT_08_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)

-- Initialise the mixer with 8 motors
assert(
    MotorsMatrix:init(8),
    "Failed to init MotorsMatrix (check Q_FRAME_CLASS=15 / scripting matrix)"
)

-- Set a custom frame identifier string for this motor layout
motors:set_frame_string("NewtonTwoV1_11022026")

-- High-visibility message so you can see the matrix was applied
gcs:send_text(MAV_SEVERITY_EMERGENCY, "LUA: MotorMatrix initialized NewtonTwoV1")