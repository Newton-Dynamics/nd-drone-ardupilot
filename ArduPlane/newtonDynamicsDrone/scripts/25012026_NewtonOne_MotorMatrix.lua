-- NewtonOneV1
-- Updated 21 Jan 2025
-- Author: Hussein Sleiman
-- Custom Motor Matrix Lua

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
-- 1 3 425
-- 3 8 435
-- 8 4 670
-- 5 7 425
-- 7 6 435
-- 6 2 670
-- 1 3 v
-- 3 8 v
-- 8 4 v
-- 5 7 v
-- 7 6 v
-- 6 2 v
-- 5 1 800
-- 7 3 800
-- 6 8 800
-- 2 4 800
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
local NEWTON_MOT_01_X = -0.484
local NEWTON_MOT_01_Y =  0.852

local NEWTON_MOT_02_X =  0.484
local NEWTON_MOT_02_Y = -1.000

local NEWTON_MOT_03_X = -0.484
local NEWTON_MOT_03_Y =  0.337

local NEWTON_MOT_04_X = -0.484
local NEWTON_MOT_04_Y = -1.000

local NEWTON_MOT_05_X =  0.484
local NEWTON_MOT_05_Y =  0.852

local NEWTON_MOT_06_X =  0.484
local NEWTON_MOT_06_Y = -0.189

local NEWTON_MOT_07_X =  0.484
local NEWTON_MOT_07_Y =  0.337

local NEWTON_MOT_08_X = -0.484
local NEWTON_MOT_08_Y = -0.189

----------------------------------------------------------------------
-- One-shot motor matrix initialisation at script load
----------------------------------------------------------------------

-- Define all 8 motors: roll, pitch, yaw_factor, test_order "ADJUSTED CG AND CANARD"
MotorsMatrix:add_motor_raw(M1, NEWTON_MOT_01_X, NEWTON_MOT_01_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1)
MotorsMatrix:add_motor_raw(M2, NEWTON_MOT_02_X, NEWTON_MOT_02_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5)
MotorsMatrix:add_motor_raw(M3, NEWTON_MOT_03_X, NEWTON_MOT_03_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
MotorsMatrix:add_motor_raw(M4, NEWTON_MOT_04_X, NEWTON_MOT_04_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4)
MotorsMatrix:add_motor_raw(M5, NEWTON_MOT_05_X, NEWTON_MOT_05_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8)
MotorsMatrix:add_motor_raw(M6, NEWTON_MOT_06_X, NEWTON_MOT_06_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6)
MotorsMatrix:add_motor_raw(M7, NEWTON_MOT_07_X, NEWTON_MOT_07_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7)
MotorsMatrix:add_motor_raw(M8, NEWTON_MOT_08_X, NEWTON_MOT_08_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)

--[[
-- Define all 8 motors: roll, pitch, yaw_factor, test_order "Older Model"
MotorsMatrix:add_motor_raw(M1, -0.441,  0.817,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1)
MotorsMatrix:add_motor_raw(M2,  0.441, -1.000,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5)
MotorsMatrix:add_motor_raw(M3, -0.441,  0.398,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
MotorsMatrix:add_motor_raw(M4, -0.441, -1.000,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4)
MotorsMatrix:add_motor_raw(M5,  0.441,  0.817,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8)
MotorsMatrix:add_motor_raw(M6,  0.441, -0.215,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6)
MotorsMatrix:add_motor_raw(M7,  0.441,  0.398,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7)
MotorsMatrix:add_motor_raw(M8, -0.441, -0.215,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
]]--
-- Initialise the mixer with 8 motors
assert(
    MotorsMatrix:init(8),
    "Failed to init MotorsMatrix (check Q_FRAME_CLASS=15 / scripting matrix)"
)

-- Set a custom frame identifier string for this motor layout
motors:set_frame_string("NewtonOneV1_25012026")

-- High-visibility message so you can see the matrix was applied
gcs:send_text(MAV_SEVERITY_EMERGENCY, "LUA: MotorMatrix initialized NewtonOne")