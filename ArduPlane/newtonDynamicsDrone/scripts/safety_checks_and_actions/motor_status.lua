-- Author: Hussein Sleiman
-- DroneCAN ESC monitor for QuadPlane
-- Release: April 1, 2026

-- ============================================================ Description ===========================================================
-- 1) One-time ESC status print after boot
-- 2) Blocks arming if any monitored ESC is not operational
-- 3) In flight, warns "Motor loss" if any monitored ESC RPM stays below threshold
--
-- Requirements:
-- - Motor servo output are mapped from Servo 1 to Servo 8
-- - This script must KEEP RUNNING for the motor-loss warning.
-- - No status is published after the initial report; it only prints on state changes (e.g. motor loss during flight or before arming).
-- ====================================================================================================================================

local SCRIPT_NAME = "motor_status"

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

local UPDATE_MS               = 200  -- script update 5 Hz
local STARTUP_WAIT_MS         = 5000 -- script waiting time to be executed
local STALE_MS                = 3000 

local RPM_MIN                 = 800
local RPM_CONSECUTIVE_COUNT   = 3    -- it requires 3 consecutive low-RPM samples to execute the warning
local RPM_MONITOR_ARM_DELAYMS = 3000 -- wait after arming before RPM fault logic starts

-- esc telem_index to droneCAN mapping
local ESC_MAP = {
    { telem_index = 0, CAN_id = 11, name = "M1" },
    { telem_index = 1, CAN_id = 12, name = "M2" },
    { telem_index = 2, CAN_id = 13, name = "M3" },
    { telem_index = 3, CAN_id = 14, name = "M4" },
    { telem_index = 4, CAN_id = 15, name = "M5" },
    { telem_index = 5, CAN_id = 16, name = "M6" },
    { telem_index = 6, CAN_id = 17, name = "M7" },
    { telem_index = 7, CAN_id = 18, name = "M8" },
    { telem_index = 8, CAN_id = 19, name = "M9" },
    { telem_index = 9, CAN_id = 20, name = "M10" },
}

local started = false
local start_ms = 0
local auth_id = nil

local initial_report_sent = false
local last_network_signature = nil
local last_motor_loss_signature = nil

local was_armed = false
local armed_since_ms = 0

local low_rpm_count = {}
for i = 1, #ESC_MAP do
    low_rpm_count[i] = 0
end

local function to_lua_number(v)
    if v == nil then
        return nil
    end

    if type(v) == "number" then
        return v
    end

    if type(v) == "userdata" then
        local ok, out = pcall(function() return v:toint() end)
        if ok and out ~= nil then
            return out
        end

        ok, out = pcall(function() return v:tofloat() end)
        if ok and out ~= nil then
            return out
        end
    end

    local n = tonumber(v)
    if n ~= nil then
        return n
    end

    return nil
end

local function fmt_num(v, pattern)
    local n = to_lua_number(v)
    if n == nil then
        return "n/a"
    end
    return string.format(pattern, n)
end

local function join(list, sep)
    return table.concat(list, sep or ", ")
end

local function ids_from_esc_list(list)
    local out = {}
    for _, esc in ipairs(list) do
        table.insert(out, tostring(esc.node_id))
    end
    return out
end

local function names_from_esc_list(list)
    local out = {}
    for _, esc in ipairs(list) do
        table.insert(out, string.format("%d %s", esc.node_id, esc.name))
    end
    return out
end

local function read_esc(idx)
    local last_ms = 0
    if esc_telem then
        last_ms = to_lua_number(esc_telem:get_last_telem_data_ms(idx)) or 0
    end

    local ok_rpm, rpm = false, nil
    local ok_v, volt = false, nil
    local ok_c, curr = false, nil
    local ok_t, temp = false, nil

    if esc_telem then
        ok_rpm, rpm = esc_telem:get_rpm(idx)
        ok_v, volt  = esc_telem:get_voltage(idx)
        ok_c, curr  = esc_telem:get_current(idx)
        ok_t, temp  = esc_telem:get_temperature(idx)
    end

    return {
        last_ms = last_ms,
        rpm  = ok_rpm and to_lua_number(rpm) or nil,
        volt = ok_v and to_lua_number(volt) or nil,
        curr = ok_c and to_lua_number(curr) or nil,
        temp = ok_t and to_lua_number(temp) or nil,
    }
end

local function evaluate_escs(now_ms)
    local all_active = true
    local missing = {}
    local data = {}

    for i, esc in ipairs(ESC_MAP) do
        local d = read_esc(esc.telem_index)
        local age = 999999999

        if d.last_ms > 0 then
            age = now_ms - d.last_ms
        end

        local active = (d.last_ms > 0) and (age <= STALE_MS)

        local item = {
            list_index   = i,
            telem_index  = esc.telem_index,
            node_id      = esc.node_id,
            name         = esc.name,
            last_ms      = d.last_ms,
            age          = age,
            active       = active,
            rpm          = d.rpm,
            volt         = d.volt,
            curr         = d.curr,
            temp         = d.temp
        }

        table.insert(data, item)

        if not active then
            all_active = false
            table.insert(missing, esc)
        end
    end

    return all_active, data, missing
end

local function send_initial_report(all_active, data, missing)
    for _, d in ipairs(data) do
        if d.active then
            gcs:send_text(
                MAV_SEVERITY.INFO,
                string.format(
                    "DCAN ID %d %s : ACTIVE | idx=%d rpm=%s V=%s I=%s T=%s",
                    d.node_id,
                    d.name,
                    d.telem_index,
                    fmt_num(d.rpm, "%.0f"),
                    fmt_num(d.volt, "%.2f"),
                    fmt_num(d.curr, "%.2f"),
                    fmt_num((d.temp ~= nil) and (d.temp / 100.0) or nil, "%.1f")
                )
            )
        else
            gcs:send_text(
                MAV_SEVERITY.WARNING,
                string.format(
                    "DCAN ID %d %s : NOT ACTIVE | idx=%d",
                    d.node_id,
                    d.name,
                    d.telem_index
                )
            )
        end
    end

    if all_active then
        gcs:send_text(MAV_SEVERITY.NOTICE, "DroneCAN ESC OPERATIONAL")
    else
        gcs:send_text(
            MAV_SEVERITY.WARNING,
            "DroneCAN NOT ACTIVE: " .. join(names_from_esc_list(missing), ", ")
        )
    end
end

local function update_aux_auth(now_ms, all_active, missing)
    if not auth_id then
        return
    end

    if (now_ms - start_ms) < STARTUP_WAIT_MS then
        arming:set_aux_auth_failed(auth_id, "ESC check pending")
        return
    end

    if all_active then
        arming:set_aux_auth_passed(auth_id)
    else
        arming:set_aux_auth_failed(auth_id, "ESC offline: " .. join(ids_from_esc_list(missing), ","))
    end
end

local function network_signature(all_active, missing)
    if all_active then
        return "OK"
    end
    return "BAD:" .. join(ids_from_esc_list(missing), ",")
end

local function in_vtol_relevant_phase()
    if not arming:is_armed() then
        return false
    end

    if not vehicle:get_likely_flying() then
        return false
    end

    if quadplane then
        if quadplane:in_vtol_mode() then
            return true
        end
        if quadplane:in_assisted_flight() then
            return true
        end
    end

    return false
end

local function clear_low_rpm_state()
    for i = 1, #ESC_MAP do
        low_rpm_count[i] = 0  --esc motor low rpm counter 
    end
    last_motor_loss_signature = nil
end

local function check_motor_loss(now_ms, data)
    if not in_vtol_relevant_phase() then
        clear_low_rpm_state()
        return
    end

    if (now_ms - armed_since_ms) < RPM_MONITOR_ARM_DELAYMS then
        clear_low_rpm_state()
        return
    end

    local low_list = {}

    for _, d in ipairs(data) do
        local low_now = false

        if d.active and d.rpm ~= nil and d.rpm < RPM_MIN then
            low_now = true
        end

        if low_now then
            low_rpm_count[d.list_index] = low_rpm_count[d.list_index] + 1
        else
            low_rpm_count[d.list_index] = 0
        end

        if low_rpm_count[d.list_index] >= RPM_CONSECUTIVE_COUNT then
            table.insert(low_list, string.format("%d(%s)", d.node_id, fmt_num(d.rpm, "%.0f")))
        end
    end

    if #low_list > 0 then
        local sig = join(low_list, ",")
        if sig ~= last_motor_loss_signature then
            gcs:send_text(MAV_SEVERITY.CRITICAL, "Motor loss: " .. join(low_list, ", "))
            last_motor_loss_signature = sig
        end
    else
        if last_motor_loss_signature ~= nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, "Motor loss cleared")
        end
        last_motor_loss_signature = nil
    end
end

function update()
    local now = to_lua_number(millis()) or 0

    if not started then
        if not esc_telem then
            gcs:send_text(MAV_SEVERITY.WARNING, "DCAN monitor: ESC telemetry API not available")
            return update, 2000
        end

        auth_id = arming:get_aux_auth_id()
        start_ms = now
        started = true
        gcs:send_text(MAV_SEVERITY.NOTICE, "DCAN ESC monitor started")
    end

    local armed = arming:is_armed()
    if armed and not was_armed then
        armed_since_ms = now
        clear_low_rpm_state()
    elseif (not armed) and was_armed then
        clear_low_rpm_state()
    end
    was_armed = armed

    local all_active, data, missing = evaluate_escs(now)

    update_aux_auth(now, all_active, missing)

    if (now - start_ms) >= STARTUP_WAIT_MS and not initial_report_sent then
        send_initial_report(all_active, data, missing)
        initial_report_sent = true
        last_network_signature = network_signature(all_active, missing)
    end

    if initial_report_sent then
        local sig = network_signature(all_active, missing)
        if sig ~= last_network_signature then
            if all_active then
                gcs:send_text(MAV_SEVERITY.NOTICE, "DroneCAN Active")
            else
                gcs:send_text(
                    MAV_SEVERITY.WARNING,
                    "DroneCAN NOT ACTIVE: " .. join(names_from_esc_list(missing), ", ")
                )
            end
            last_network_signature = sig
        end
    end

    check_motor_loss(now, data)

    return update, UPDATE_MS
end

function protected_wrapper()
    local ok, ret1, ret2 = pcall(update)
    if not ok then
        gcs:send_text(MAV_SEVERITY.EMERGENCY, "DCAN script error: " .. tostring(ret1))
        return protected_wrapper, 1000
    end
    return ret1, ret2
end

return protected_wrapper()