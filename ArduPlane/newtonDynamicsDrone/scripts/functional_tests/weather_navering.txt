-- Author: Hussein Sleiman
-- Release: April 27, 2026

-- ================================================================================= Description ===========================================================================================
-- This script is to enable/disable the weathervaning feature in ArduPlane VTOLs using an RC switch. 
-- It allows pilots to quickly toggle weathervaning on or off without needing to adjust multiple parameters manually. 
-- The weathervaning behavior can be take effect only in QLOITER, QRTL, and AUTO VTOL modes. 

-- Sets of parameters are defined for both the ON and OFF states. When the assigned RC switch is toggled, the script applies the corresponding profile by updating the relevant parameters.
-- The parameters can be adjusted manually for fine-tuning the weathervaning behavior, but the script provides a convenient way to switch between the two states during flight.

-- PARAMETERS:
-- Q_WVANE_ENABLE       enable and define weathervane direction
-- Q_WVANE_GAIN         yaw response strength
-- Q_WVANE_HGT_MIN      minimum height before allowed
-- Q_WVANE_SPD_MAX      only below this groundspeed
-- Q_WVANE_VELZ_MAX     only below this vertical speed
-- Q_WVANE_TAKEOFF      override during AUTO VTOL takeoff
-- Q_WVANE_LAND         override during AUTO VTOL landing   

-- To enable the weathervaning defined features assign an RC channel to option 300 (e.g. CH5) and use the switch to toggle between the ON and OFF profiles
-- ========================================================================================================================================================================================

local SCRIPT_NAME = "weathervane_switch"

-- ========= User configuration =========
local TRIG_CH      = 9      -- RC channel used to activate/deactivate weathervaning
local TRIG_HIGH_US = 1800   -- PWM threshold to enable weathervaning
local TRIG_LOW_US  = 1200   -- PWM threshold to disable weathervaning
local UPDATE_MS    = 250

-- Weathervaning ON profile
local WVANE_ON = {
    Q_WVANE_ENABLE   = 1,     -- enable weathervaning
    Q_WVANE_GAIN     = 3.0,   -- yaw response strength
    Q_WVANE_ANG_MIN  = 1.0,   -- minimum roll/pitch angle before acting

    Q_WVANE_HGT_MIN  = 0.0,   -- minimum height before allowed
    Q_WVANE_SPD_MAX  = 0.0,   -- only active below this groundspeed
    Q_WVANE_VELZ_MAX = 0.0,   -- only active below this vertical speed

    Q_WVANE_OPTIONS  = 0,     -- default behavior
    Q_WVANE_TAKEOFF  = 0,     -- AUTO VTOL takeoff behavior
    Q_WVANE_LAND     = 0,     -- AUTO VTOL landing behavior
}

-- Weathervaning OFF profile
local WVANE_OFF = {
    Q_WVANE_ENABLE = 0,
}
-- ======================================
local SEV_INFO    = 6
local SEV_WARNING = 4
local last_state = nil

local function gcs_msg(severity, txt)
    gcs:send_text(severity, string.format("%s: %s", SCRIPT_NAME, txt))
end

local function rcpwm(ch)
    local v = rc:get_pwm(ch)
    return tonumber(v) or 0
end

local function apply_profile(profile)
    for name, value in pairs(profile) do
        param:set(name, value)
    end
end

function update()
    local pwm = rcpwm(TRIG_CH)

    if pwm == 0 then
        gcs_msg(SEV_WARNING, "No RC input on CH" .. tostring(TRIG_CH))
        return update, 1000
    end

    -- Switch HIGH: enable weathervaning
    if pwm >= TRIG_HIGH_US then
        if last_state ~= "ON" then
            apply_profile(WVANE_ON)
            gcs_msg(SEV_INFO, "Weathervaning ON")
            last_state = "ON"
        end

    -- Switch LOW: disable weathervaning
    elseif pwm <= TRIG_LOW_US then
        if last_state ~= "OFF" then
            apply_profile(WVANE_OFF)
            gcs_msg(SEV_INFO, "Weathervaning OFF")
            last_state = "OFF"
        end
    end

    return update, UPDATE_MS
end

gcs_msg(SEV_INFO, "Init on CH" .. tostring(TRIG_CH))
return update()