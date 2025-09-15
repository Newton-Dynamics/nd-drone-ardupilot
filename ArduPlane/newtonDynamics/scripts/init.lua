-- APM/scripts/init.lua
gcs:send_text(6, "Lua init: loading custom mixers…")
-- if you want to force a specific filename to load first:
-- require("h-frame")   -- only if your file is named h-frame.lua
-- otherwise do nothing: all .lua files get loaded anyway
return
