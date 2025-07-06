-------------------------------------------------------------
--  Scatter-plot of end-effector frames along the path
--  Works on CoppeliaSim ≥ 4.8 (drawing_axes was removed)
-------------------------------------------------------------
EE_NAME          = 'link6'     -- exact scene name of your tool link
AXIS_LENGTH      = 0.10        -- triad axis length  (metres)
LINE_WIDTH_PX    = 2           -- line thickness     (pixels)
TRANSPARENT      = false       -- true  ➜ 50 % alpha
PLOT_THRESHOLD   = 0.02        -- metres: draw new triad only if
                               -- EE moved farther than this
-------------------------------------------------------------
-- you can change the colours here if you like
RED   = {1,0,0}
GREEN = {0,1,0}
BLUE  = {0,0,1}
-------------------------------------------------------------

-- drawing flags ------------------------------------------------------
local DRAW_FLAGS = sim.drawing_lines              -- 3-D line segments
if TRANSPARENT then
    DRAW_FLAGS = DRAW_FLAGS + sim.drawing_50percenttransparency
end
-- (no sim.drawing_local ⇒ vertices are absolute world coordinates)

----------------------------------------------------------------------
-- INITIALISATION -----------------------------------------------------
----------------------------------------------------------------------
function sysCall_init()
    eeHandle = sim.getObjectHandle(EE_NAME)
    if eeHandle == -1 then
        error("Object '"..EE_NAME.."' not found in the scene")
    end

    -- one drawing object per axis, each with its own default colour
    xDraw = sim.addDrawingObject(DRAW_FLAGS, LINE_WIDTH_PX, 0, -1, -1, RED)
    yDraw = sim.addDrawingObject(DRAW_FLAGS, LINE_WIDTH_PX, 0, -1, -1, GREEN)
    zDraw = sim.addDrawingObject(DRAW_FLAGS, LINE_WIDTH_PX, 0, -1, -1, BLUE)

    lastPlotPos = nil          -- no triad plotted yet
    thr2 = PLOT_THRESHOLD*PLOT_THRESHOLD   -- compare squared distance
end

----------------------------------------------------------------------
-- SENSING STEP: plot triad if EE moved enough -----------------------
----------------------------------------------------------------------
function sysCall_sensing()
    local m = sim.getObjectMatrix(eeHandle, -1)          -- 12-el world matrix
    local p = {m[4], m[8], m[12]}                        -- current world pos

    -- decide whether to add a new frame -----------------------------
    local needPlot = false
    if not lastPlotPos then
        needPlot = true                                  -- first ever plot
    else
        local dx = p[1]-lastPlotPos[1]
        local dy = p[2]-lastPlotPos[2]
        local dz = p[3]-lastPlotPos[3]
        needPlot = (dx*dx + dy*dy + dz*dz) >= thr2       -- moved far enough?
    end
    if not needPlot then return end          -- nothing to draw this step

    -- endpoints of the three axes in world coordinates --------------
    local xEnd = {p[1] + AXIS_LENGTH*m[1],
                  p[2] + AXIS_LENGTH*m[5],
                  p[3] + AXIS_LENGTH*m[9]}
    local yEnd = {p[1] + AXIS_LENGTH*m[2],
                  p[2] + AXIS_LENGTH*m[6],
                  p[3] + AXIS_LENGTH*m[10]}
    local zEnd = {p[1] + AXIS_LENGTH*m[3],
                  p[2] + AXIS_LENGTH*m[7],
                  p[3] + AXIS_LENGTH*m[11]}

    -- add the three coloured segments -------------------------------
    sim.addDrawingObjectItem(xDraw, {p[1],p[2],p[3],  xEnd[1],xEnd[2],xEnd[3]})
    sim.addDrawingObjectItem(yDraw, {p[1],p[2],p[3],  yEnd[1],yEnd[2],yEnd[3]})
    sim.addDrawingObjectItem(zDraw, {p[1],p[2],p[3],  zEnd[1],zEnd[2],zEnd[3]})

    lastPlotPos = p            -- remember where we just plotted
end

----------------------------------------------------------------------
-- CLEAN-UP -----------------------------------------------------------
----------------------------------------------------------------------
function sysCall_cleanup()
    sim.removeDrawingObject(xDraw)
    sim.removeDrawingObject(yDraw)
    sim.removeDrawingObject(zDraw)
end
