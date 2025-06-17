function sysCall_init()
    sim   = require('sim')
    simUI = require('simUI')
    
    baseHandle = sim.getObject('/frame_base')
    base = sim.getObjectPosition(baseHandle, -1)
    base = {base[1], base[2], base[3]}
  
    -- handle do link
    linkHandle = sim.getObject('/link6/tcp/frame_tcp')
    if linkHandle == -1 then
        error("could not find link 'link6'")
    end
  
    -- initial time
    startTime = sim.getSimulationTime()
  
    -- UI with two labels (position and orientation)
    local xml = [[
        <ui title="eff_pose (from base)" layout="vbox">
            <label id="11" text="pos: "/>
            <label id="12" text="ori: "/>
        </ui>
    ]]
    ui = simUI.create(xml)
    sim.addLog(sim.verbosity_scriptinfos, string.format("Started eff_pose script !"))
    sim.addLog(sim.verbosity_scriptinfos, string.format("Base position: [%.3f,%.3f,%.3f]", base[1],base[2],base[3]))
  end
  
  function sysCall_sensing()
    -- pose reading
    local pos = sim.getObjectPosition    (linkHandle, -1)   -- {x,y,z}
    local o = sim.getObjectOrientation(linkHandle, -1)   -- {roll,pitch,yaw}
    
    -- simple transform with respect to the base frame
    pos_aux = {pos[1], pos[2], pos[3]}
    local p = {pos[1] - base[1], pos[2] - base[2], pos[3] - base[3]}
  
    -- update labels
    simUI.setLabelText(ui, 11,
        string.format('pos: %.4f, %.4f, %.4f', p[1], p[2], p[3]), true)
    simUI.setLabelText(ui, 12,
        string.format('ori: %.3f, %.3f, %.3f', o[1], o[2], o[3]), true)
  end
  
  function sysCall_cleanup()
    if ui then
        simUI.destroy(ui)
        ui = nil
    end
  end
  