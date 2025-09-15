-- NEWTON Hummingbird 2024/06/17
-- 06/17/2024 - initial 
-- 06/18/2024 - changed beam to beam distance to 500
-- 06/18/2024 - changed to 10 Motors

local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_EMERGENCY = 0

--- because flags
local IS_INITIALISED = false

-- duplicate the #defines from AP_Motors
local AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1
local AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1

-- MOTOR MATRIX NUMBERING and COORDINATES
-- 515 beam to beam
-- 70mm backwards

-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_01, -1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_02,  1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_03, -1.0,  0.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_04, -1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 )

-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_05,  1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_06,  1.0, -0.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_07,  1.0,  0.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 )
-- MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_08, -1.0, -0.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 )

-- NewtonOne:
-- https://www.iforce2d.net/mixercalc/
-- 1 3 v
-- 3 8 v
-- 8 4 v
-- 5 7 v
-- 7 6 v
-- 6 2 v
-- 5 1 800
-- 5 7 345
-- 1 3 345
-- 7 6 640
-- 3 8 640
-- 6 2 665
-- 8 4 665
-- 5 1 h
-- 7 3 h
-- 6 8 h
-- 2 4 h

-- NewtonOne 14":
-- 1 3 v
-- 3 8 v
-- 8 4 v
-- 5 7 v
-- 7 6 v
-- 6 2 v
-- 5 1 800
-- 5 7 349
-- 1 3 349
-- 7 6 638
-- 3 8 638
-- 6 2 665
-- 8 4 665
-- 5 1 h
-- 7 3 h
-- 6 8 h
-- 2 4 h

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Newton SITL, Dimensions are measured from the Blender File 
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- 3 1 v
-- 3 8 v
-- 8 4 v
-- 5 7 v
-- 7 6 v
-- 6 2 v
-- 5 1 320.477
-- 5 7 152.124
-- 1 3 152.124
-- 7 6 226.986
-- 3 8 226.986
-- 6 2 284.225
-- 8 4 284.225
-- 5 1 h
-- 7 3 h
-- 6 8 h
-- 2 4 h
--------------------------------------------------------------------------------
-- Output: ArduCopter Motor Matrix
--------------------------------------------------------------------------------
-- add_motor_raw(AP_MOTORS_MOT_1, -0.44, 0.819, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
-- add_motor_raw(AP_MOTORS_MOT_2, 0.44, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
-- add_motor_raw(AP_MOTORS_MOT_3, -0.44, 0.401, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
-- add_motor_raw(AP_MOTORS_MOT_4, -0.44, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
-- add_motor_raw(AP_MOTORS_MOT_5, 0.44, 0.819, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
-- add_motor_raw(AP_MOTORS_MOT_6, 0.44, -0.22, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
-- add_motor_raw(AP_MOTORS_MOT_7, 0.44, 0.401, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7);
-- add_motor_raw(AP_MOTORS_MOT_8, -0.44, -0.22, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8);
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

-- add_motor_raw(AP_MOTORS_MOT_1, -0.442, 0.823, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
-- add_motor_raw(AP_MOTORS_MOT_2, 0.442, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
-- add_motor_raw(AP_MOTORS_MOT_3, -0.442, 0.442, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
-- add_motor_raw(AP_MOTORS_MOT_4, -0.442, -1, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
-- add_motor_raw(AP_MOTORS_MOT_5, 0.442, 0.823, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
-- add_motor_raw(AP_MOTORS_MOT_6, 0.442, -0.265, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
-- add_motor_raw(AP_MOTORS_MOT_7, 0.442, 0.442, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7);
-- add_motor_raw(AP_MOTORS_MOT_8, -0.442, -0.265, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8);

local AP_MOTORS_MOT_01 = 0
local NEWTON_MOT_01_X = -0.44
local NEWTON_MOT_01_Y = 0.819

local AP_MOTORS_MOT_02 = 1
local NEWTON_MOT_02_X = 0.44
local NEWTON_MOT_02_Y = -1

local AP_MOTORS_MOT_03 = 2
local NEWTON_MOT_03_X = -0.44
local NEWTON_MOT_03_Y = 0.401

local AP_MOTORS_MOT_04 = 3
local NEWTON_MOT_04_X = -0.44
local NEWTON_MOT_04_Y = -1

local AP_MOTORS_MOT_05 = 4
local NEWTON_MOT_05_X = 0.44
local NEWTON_MOT_05_Y = 0.819

local AP_MOTORS_MOT_06 = 5
local NEWTON_MOT_06_X = 0.44
local NEWTON_MOT_06_Y = -0.22

local AP_MOTORS_MOT_07 = 6
local NEWTON_MOT_07_X = 0.44
local NEWTON_MOT_07_Y = 0.401

local AP_MOTORS_MOT_08 = 7
local NEWTON_MOT_08_X = -0.44
local NEWTON_MOT_08_Y = -0.22


local CX_CURRENT_TOP_THRUST_RATIO = 1.0
local CX_CURRENT_TOP_YAW_RATIO = 1.0

local PARAM_TABLE_KEY = 1
assert(param:add_table(PARAM_TABLE_KEY, "CX_", 10), 'could not add CX_ table')
assert(param:add_param(PARAM_TABLE_KEY, 1,  'TOP_THRUST', 1.0), 'could not add TOP_THRUST')
assert(param:add_param(PARAM_TABLE_KEY, 2,  'TOP_YAW', 1.0), 'could not add TOP_YAW')

local param_CX_TOP_THRUST_RATIO = Parameter()
local param_CX_TOP_YAW_RATIO = Parameter()
assert(param_CX_TOP_THRUST_RATIO:init("CX_TOP_THRUST"), "Could not find CX_TOP_THRUST")
assert(param_CX_TOP_YAW_RATIO:init("CX_TOP_YAW"), "Could not find CX_TOP_YAW")

-- helper function duplication of the one found in AP_MotorsMatrix
local function add_motor(motor_num, angle_degrees, yaw_factor, testing_order)
    roll_factor = math.cos(math.rad(angle_degrees + 90));
    pitch_factor = math.cos(math.rad(angle_degrees));
    
    -- vvoid AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor)
    MotorsMatrix:add_motor_raw(motor_num,roll_factor, pitch_factor, yaw_factor, testing_order)
    gcs:send_text(0, "MOT#:" .. motor_num .. ", [deg]:" .. angle_degrees .. ", [yaw]: " .. yaw_factor .. ", [test_order]: " .. testing_order)    

end

local function updateThrustFactors()
    -- this duplicates the add motor format used in AP_Motors for ease of modification of existing mixes
    -- standard octa quad X mix

    local CX_NEW_TOP_THRUST_RATIO = param_CX_TOP_THRUST_RATIO:get()
    local CX_NEW_TOP_YAW_RATIO = param_CX_TOP_YAW_RATIO:get()

    -- only if we are sure it's been initialized do we exit early when the parameters are unchanged.
    if IS_INITIALISED then
        -- if unchanged, return
        if CX_NEW_TOP_THRUST_RATIO==CX_CURRENT_TOP_THRUST_RATIO and CX_NEW_TOP_YAW_RATIO==CX_CURRENT_TOP_YAW_RATIO then
            --no action required check back in, in 5 seconds
            return updateThrustFactors, 5000
        end
        -- if out of range, return
        if (CX_NEW_TOP_THRUST_RATIO>1) or (CX_NEW_TOP_THRUST_RATIO<0) or (CX_NEW_TOP_YAW_RATIO>1) or (CX_NEW_TOP_YAW_RATIO<0) then
            gcs:send_text(6, "WARNING: Tried to set COAX value out of range [0,1], skipping...")
            return updateThrustFactors, 5000
        end
        -- TODO if armed return...
    end

    CX_CURRENT_TOP_THRUST_RATIO = CX_NEW_TOP_THRUST_RATIO
    CX_CURRENT_TOP_YAW_RATIO = CX_NEW_TOP_YAW_RATIO

    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_01, NEWTON_MOT_01_X, NEWTON_MOT_01_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_02, NEWTON_MOT_02_X, NEWTON_MOT_02_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_03, NEWTON_MOT_03_X, NEWTON_MOT_03_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_04, NEWTON_MOT_04_X, NEWTON_MOT_04_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 )

    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_05, NEWTON_MOT_05_X, NEWTON_MOT_05_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_06, NEWTON_MOT_06_X, NEWTON_MOT_06_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_07, NEWTON_MOT_07_X, NEWTON_MOT_07_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 )
    MotorsMatrix:add_motor_raw(AP_MOTORS_MOT_08, NEWTON_MOT_08_X, NEWTON_MOT_08_Y, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 ) 

 
    -- -- setup throttle factors
    -- -- top motors reduce throttle
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_01, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_02, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_03, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_04, 1.0)

    -- bottom motors get full (the default is one so these lines don't actually do anything.)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_05, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_06, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_07, 1.0)
    MotorsMatrix:set_throttle_factor(AP_MOTORS_MOT_08, 1.0)
	
	gcs:send_text(MAV_SEVERITY_EMERGENCY, "LUA: MotorMatrix - NewtonOne Vertical - V0.0.2 - 20250514")


    -- assert(MotorsMatrix:init(8), "Failed to init MotorsMatrix")
    assert(MotorsMatrix:init(8), "Failed to init MotorsMatrix - Check Q_FRAME_CLASS==15")
    motors:set_frame_string("NewtonOne_20250717")

    gcs:send_text(MAV_SEVERITY_EMERGENCY, "LUA: MotorMatrix - NO-V-V0.0.2 - 20250514")

    IS_INITIALISED = true

    return updateThrustFactors, 5000
end

return updateThrustFactors, 1000

