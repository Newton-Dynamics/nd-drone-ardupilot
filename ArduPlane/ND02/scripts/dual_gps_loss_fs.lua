-- Author: Hussein Sleiman

-- Drone Version: NewtonTwo - Series 1 (ND21)
-- Published in 06 August 2026
-- Updated in 13 August 2026

------------------------------------------------------------------------------------------ Description -----------------------------------------------------------------------------------------------
-- This script implements a failsafe for dual GPS loss detection in QuadPlane position control mode. 
-- It monitors the health of both GPS sensors and triggers a failsafe mode (QHOVER) if both GPS units are lost for a specified duration (ALL_GPS_LOSS_DEBOUNCE_MS). The monitor sampling rate 
-- UPDATE_PERIOD_MS is based on GPS1_MS_RATE and GPS2_MS_RATE parameters, which are set to 10Hz in this case. Please adjust the value if the GPS update rate is changed. 
-- The script also handles recovery scenarios, ensuring that the system only exits failsafe mode after a confirmed recovery period (GPS_RECOVERY_CONFIRM_MS).
-- In addition the script checks GPS_AUTO_SWITCH parameter if set to 4 (bit 4 in ArduPlane v4.6.3) means it uses the primary GPS, that is GPS1 in our case and if it failed then use the second GPS.

-- In QLOITER or AUTO VTOL modes, the script monitors both GPS sensors. If both failaed to provide a valid fix, the script will trigger QHOVER mode, allowing the pilot to be in control. 
-- A warning and notificiation will pop up for GPS loss and mode change. 
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ 

local SCRIPT_NAME = "GPSFS"
local DEPLOYMENT_NAME = "Dual GPS Loss FS"

-- ---------------- User settings ----------------
local UPDATE_PERIOD_MS             = 100   -- 10 Hz script update
local GPS_MESSAGE_TIMEOUT_MS       = 2000  -- latest GPS message must be newer
local GPS_FIX_TIMEOUT_MS           = 2000  -- latest valid fix must be newer
local GPS_FAIL_REPORT_DELAY_MS     = 500   -- debounce individual failure messages
local GPS_RECOVERY_CONFIRM_MS      = 1000  -- recovered GPS must remain usable
local ALL_GPS_LOSS_DEBOUNCE_MS     = 3000  -- both GPS must remain unavailable
local MODE_CHANGE_RETRY_MS         = 1000

-- ArduPlane 4.6.3 mode numbers
local MODE_AUTO    = 10
local MODE_QHOVER  = 18
local MODE_QLOITER = 19

-- MAV_SEVERITY values
local SEVERITY_CRITICAL = 2
local SEVERITY_ERROR    = 3
local SEVERITY_WARNING  = 4
local SEVERITY_NOTICE   = 5
local SEVERITY_INFO     = 6

local gps_auto_switch = param:get("GPS_AUTO_SWITCH")

if gps_auto_switch == nil or gps_auto_switch ~= 4 then
    gcs:send_text(SEVERITY_ERROR, "GPSFS: WRONG GPS_AUTO_SWITCH")
    return
end

local sensor_state = {
    { failed_reported = false, fail_since = nil, recovery_since = nil },
    { failed_reported = false, fail_since = nil, recovery_since = nil },
}

local all_loss_since = nil
local any_recovery_since = nil
local last_mode_attempt_ms = nil
local failsafe_latched = false
local was_armed = false

local function elapsed_ms(now, then_ms)
    return (now - then_ms):tofloat()
end

local function send_text(severity, text)
    gcs:send_text(severity, SCRIPT_NAME .. ": " .. text)
end

-- Returns:
--   usable: true only with 3D-or-better fix and fresh data
--   reason: concise reason when unusable
local function gps_usable(instance, now)
    local count = gps:num_sensors()
    if count <= instance then
        return false, "not detected"
    end

    local status = gps:status(instance)
    if status < gps.GPS_OK_FIX_3D then
        return false, string.format("fix=%d", status)
    end

    local last_message = gps:last_message_time_ms(instance)
    local message_age = elapsed_ms(now, last_message)
    if message_age > GPS_MESSAGE_TIMEOUT_MS then
        return false, string.format("msg stale %.1fs", message_age * 0.001)
    end

    local last_fix = gps:last_fix_time_ms(instance)
    local fix_age = elapsed_ms(now, last_fix)
    if fix_age > GPS_FIX_TIMEOUT_MS then
        return false, string.format("fix stale %.1fs", fix_age * 0.001)
    end

    return true, "healthy"
end

local function update_sensor_message(instance, usable, reason, now)
    local state = sensor_state[instance + 1]
    local label = "GPS" .. tostring(instance + 1)

    if usable then
        state.fail_since = nil

        if state.failed_reported then
            if state.recovery_since == nil then
                state.recovery_since = now
            elseif elapsed_ms(now, state.recovery_since) >= GPS_RECOVERY_CONFIRM_MS then
                state.failed_reported = false
                state.recovery_since = nil
                send_text(SEVERITY_NOTICE, label .. " recovered")
            end
        else
            state.recovery_since = nil
        end
        return
    end

    state.recovery_since = nil

    if not state.failed_reported then
        if state.fail_since == nil then
            state.fail_since = now
        elseif elapsed_ms(now, state.fail_since) >= GPS_FAIL_REPORT_DELAY_MS then
            state.failed_reported = true
            state.fail_since = nil
            send_text(SEVERITY_WARNING, label .. " failed: " .. reason)
        end
    end
end

local function in_target_operation()
    local mode = vehicle:get_mode()

    if mode == MODE_QLOITER then
        return true
    end

    -- AUTO is targeted only while the QuadPlane is actually in VTOL operation.
    if mode == MODE_AUTO and quadplane:in_vtol_mode() then
        return true
    end

    return false
end

local function reset_pending_loss()
    all_loss_since = nil
    any_recovery_since = nil
    last_mode_attempt_ms = nil
end

local function update()
    local now = millis()
    local armed = arming:is_armed()

    -- Re-arm starts a new failsafe cycle. A triggered event remains latched
    -- for the remainder of the current armed flight.
    if not armed then
        if was_armed and failsafe_latched then
            send_text(SEVERITY_INFO, "disarmed; failsafe latch reset")
        end
        failsafe_latched = false
        reset_pending_loss()
    end
    was_armed = armed

    local gps1_ok, gps1_reason = gps_usable(0, now)
    local gps2_ok, gps2_reason = gps_usable(1, now)

    update_sensor_message(0, gps1_ok, gps1_reason, now)
    update_sensor_message(1, gps2_ok, gps2_reason, now)

    if failsafe_latched then
        return update, UPDATE_PERIOD_MS
    end

    local eligible = armed and vehicle:get_likely_flying() and in_target_operation()

    if not eligible then
        reset_pending_loss()
        return update, UPDATE_PERIOD_MS
    end

    local both_unusable = (not gps1_ok) and (not gps2_ok)

    if both_unusable then
        any_recovery_since = nil

        if all_loss_since == nil then
            all_loss_since = now
            send_text(SEVERITY_CRITICAL, "both GPS lost; debounce started")
        end

        if elapsed_ms(now, all_loss_since) >= ALL_GPS_LOSS_DEBOUNCE_MS then
            local retry_allowed = last_mode_attempt_ms == nil or
                elapsed_ms(now, last_mode_attempt_ms) >= MODE_CHANGE_RETRY_MS

            if retry_allowed then
                last_mode_attempt_ms = now
                local changed = vehicle:set_mode(MODE_QHOVER)

                if changed then
                    failsafe_latched = true
                    send_text(SEVERITY_CRITICAL, "QHOVER triggered: both GPS lost")
                else
                    send_text(SEVERITY_ERROR, "QHOVER mode change failed; retrying")
                end
            end
        end

        return update, UPDATE_PERIOD_MS
    end

    -- At least one GPS looks usable. Do not cancel a pending all-GPS-loss
    -- event until that recovery remains continuously healthy for the
    -- configured confirmation period.
    if all_loss_since ~= nil then
        if any_recovery_since == nil then
            any_recovery_since = now
        elseif elapsed_ms(now, any_recovery_since) >= GPS_RECOVERY_CONFIRM_MS then
            send_text(SEVERITY_NOTICE, "GPS recovered; QHOVER cancelled")
            reset_pending_loss()
        end
    end

    return update, UPDATE_PERIOD_MS
end

send_text(SEVERITY_INFO, "initialized (" .. DEPLOYMENT_NAME .. ")")
return update, UPDATE_PERIOD_MS
