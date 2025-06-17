-- Draws the end-effector path of /link6 with a colour that can be
-- changed at run-time through the scene-signal  signal.trajColorRGB

local LINK     = '/link6'
local MSG_NAME = "customData.trajColorRGB"      -- full property key
local WIDTH_PX = 3
local MAX_PTS  = 1e5

---------------------------------------------------------------
local function parseColor(raw)              -- "1,0,0" ? {1,0,0}
    if not raw or raw=='' then return nil end
    local c = {}
    for v in string.gmatch(raw,'[%d%.eE+-]+') do
        c[#c+1] = tonumber(v)
    end
    return (#c==3) and c or nil
end

local function printFloatTable(t)
    if t == nil then
        return
    end
    -- Unpack back into a Lua number-array
    -- for i,v in ipairs(t) do
    --     print(string.format('t[%d] = %f', i, v))
    -- end
end

local function newDrawer(col)
    return sim.addDrawingObject(
        sim.drawing_linestrip|sim.drawing_cyclic,
        WIDTH_PX, 0, -1, MAX_PTS, col)
end
---------------------------------------------------------------

function sysCall_init()
    sim   = require('sim')
    link   = sim.getObject(LINK)
    assert(link~=-1, LINK..' not found')

    currentColor = {1,1,0}                  -- start yellow
    drawer       = newDrawer(currentColor)
    lastPos      = nil --sim.getObjectPosition(link, -1)
    drawingOn    = true
    lastRaw      = nil                      -- remembers last value
    
    sim.addLog(sim.verbosity_scriptinfos, string.format("Started 3d_plot script !"))
end

function sysCall_sensing()
    -- 1 · fetch colour command (nil ? property not set)
    local myData = sim.getBufferProperty(sim.handle_scene, MSG_NAME, {noError = true})

    if myData then
        raw = sim.unpackFloatTable(myData)
        printFloatTable(raw)
        raw = table.concat(raw, ",")
        --sim.addLog(sim.verbosity_scriptinfos, string.format('raw color = %s', raw))

    end

    if raw ~= lastRaw then                  -- only react on change
        lastRaw = raw
        sim.addLog(sim.verbosity_scriptinfos, string.format('Using color = %s', raw))
        
        if raw==nil or raw=='' or raw=='OFF' or raw=='off' or raw=='0.0,0.0,0.0' then
            drawingOn = false              -- just stop adding points
        else
            local col = parseColor(raw)
            if col then
                drawingOn  = true
                currentColor = col
                drawer = newDrawer(currentColor)   -- fresh object
                lastPos = sim.getObjectPosition(link, -1)
            end
        end
    end

    -- 2 · append next segment if drawing is enabled
    if drawingOn and lastPos ~= nil then
        local p = sim.getObjectPosition(link, -1)
        sim.addDrawingObjectItem(drawer,{
            lastPos[1],lastPos[2],lastPos[3],
            p[1],      p[2],      p[3]})
        lastPos = p
    end
end

function sysCall_cleanup()
    if drawer then sim.removeDrawingObject(drawer) end
end
