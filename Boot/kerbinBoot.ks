clearscreen.
// Print "Boot sequence initiated"
print "Boot sequence initiated".
// Print "Loading boot sequence..."
print "Loading boot sequence...".
runOncePath("0:/lib/Terminal").
bxbOpenTerminal().
cd("0:/missions").
run kerbinOrbit.
bxbCloseTerminal().
