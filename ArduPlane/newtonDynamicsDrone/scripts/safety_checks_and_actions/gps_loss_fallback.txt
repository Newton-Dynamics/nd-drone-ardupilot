-- gps_loss_to_qhover.lua

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT     = 1,
    CRITICAL  = 2,
    ERROR     = 3,
    WARNING   = 4,
    NOTICE    = 5,
    INFO      = 6,
    DEBUG     = 7
}

-- ArduPlane QuadPlane mode numbers
local MODE_QHOVER = 18

-- GPS status values
-- 0 = NO_GPS
-- 1 = NO_FIX
-- 2 = GPS_OK_FIX_2D
-- 3 = GPS_OK_FIX_3D
-- 4 = GPS_OK_FIX_3D_DGPS
-- 5 = GPS_OK_FIX_3D_RTK_FLOAT
-- 6 = GPS_OK_FIX_3D_RTK_FIXED

local GPS_OK_FIX_3D = 3

-- Script timing
local UPDATE_RATE_MS = 100

-- If no GPS fix update is received for this time, treat GPS as stale/lost
local GPS_STALE_TIMEOUT_MS = 1500

-- 1 = immediate action
-- 2 or 3 = tolerate short glitches
local REQUIRED_BAD_COUNT = 1

local bad_gps_count = 0
local qhover_triggered = false
local last_msg_ms = 0

local function time_ms()
    return millis():tofloat()
end

local function send_text_limited(severity, text, interval_ms)
    local now = time_ms()

    if now - last_msg_ms > interval_ms then
        gcs:send_text(severity, text)
        last_msg_ms = now
    end
end

local function is_vehicle_flying()
    if not arming:is_armed() then
        return false
    end

    if vehicle:get_likely_flying() then
        return true
    end

    return false
end

local function gps_is_bad()
    local num_gps = gps:num_sensors()

    if num_gps == nil or num_gps <= 0 then
        return true, "no GPS sensor"
    end

    local primary = gps:primary_sensor()

    if primary == nil then
        return true, "no primary GPS"
    end

    local status = gps:status(primary)

    if status == nil then
        return true, "GPS status unavailable"
    end

    if status < GPS_OK_FIX_3D then
        return true, "GPS fix below 3D, status=" .. tostring(status)
    end

    local last_fix_ms = gps:last_fix_time_ms(primary)

    if last_fix_ms == nil then
        return true, "GPS last fix time unavailable"
    end

    local age_ms = time_ms() - last_fix_ms:tofloat()

    if age_ms > GPS_STALE_TIMEOUT_MS then
        return true, "GPS stale, age_ms=" .. tostring(math.floor(age_ms))
    end

    return false, "GPS OK"
end

local function switch_to_qhover(reason)
    local current_mode = vehicle:get_mode()

    if current_mode == MODE_QHOVER then
        qhover_triggered = true
        return
    end

    gcs:send_text(
        MAV_SEVERITY.CRITICAL,
        "GPS FAILSAFE: switching to QHOVER, reason: " .. reason
    )

    local success = vehicle:set_mode(MODE_QHOVER)

    if success then
        qhover_triggered = true
        gcs:send_text(MAV_SEVERITY.CRITICAL, "GPS FAILSAFE: QHOVER accepted")
    else
        gcs:send_text(MAV_SEVERITY.EMERGENCY, "GPS FAILSAFE: QHOVER mode change failed")
    end
end

function update()
    if not is_vehicle_flying() then
        bad_gps_count = 0
        qhover_triggered = false
        return update, UPDATE_RATE_MS
    end

    if qhover_triggered then
        return update, UPDATE_RATE_MS
    end

    local bad, reason = gps_is_bad()

    if bad then
        bad_gps_count = bad_gps_count + 1

        send_text_limited(
            MAV_SEVERITY.WARNING,
            "GPS FAILSAFE: bad GPS detected: " .. reason,
            1000
        )

        if bad_gps_count >= REQUIRED_BAD_COUNT then
            switch_to_qhover(reason)
        end
    else
        bad_gps_count = 0
    end

    return update, UPDATE_RATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "GPS FAILSAFE: QHOVER script loaded")

return update, UPDATE_RATE_MS