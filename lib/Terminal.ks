global function bxbOpenTerminal {
    core:part:getmodule("kOSProcessor"):doevent("open terminal").
}

global function bxbCloseTerminal {
    core:part:getmodule("kOSProcessor"):doevent("close terminal").
}