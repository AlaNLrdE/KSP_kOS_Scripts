// Initialize variables
set pitch to 90.
set speed_threshold to 100. // Speed threshold to start changing pitch
set apoapsis_threshold to 80000. // Apoapsis height threshold to change pitch to 0
set altitude_threshold to 10000. // Altitude threshold to complete pitch change to 45 degrees
set gravity_turn_end to 10000. // End gravity turn at 45 km

// Define stage properties directly
set lockInitialAltitude to false.

// Launch the rocket
lock throttle to 1.
stage.
//wait 1.
//stage.

set currentThrottle to throttle.
// Main loop
print "Starting gravity turn...".
// Burn until fuel runs out
until ship:apoapsis >= apoapsis_threshold 
{
    // Update pitch based on speed and apoapsis height
    if ship:velocity:surface:mag > speed_threshold and ship:apoapsis < apoapsis_threshold and altitude < altitude_threshold 
    {
        if not lockInitialAltitude 
        {
            set lockInitialAltitude to true.
            set initialAltitude to altitude.
        }
        set pitch to 90 - (altitude - initialAltitude) / (gravity_turn_end - initialAltitude) * 45.
        lock steering to heading(90, pitch).
        print "(2) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
    } 
    else if altitude >= altitude_threshold and ship:apoapsis < apoapsis_threshold 
    {
        if ship:altitude > 40000
        {
            set pitch to 15.
        }
        else
        {
            set pitch to 45.
        }
        lock steering to heading(90, pitch).
        print "(3) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
    }
    else
    {
        lock steering to heading(90, 90).
        print "(1) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°". 
    }
    if eta:apoapsis > 50 {
        if throttle > 0.1 {
            set currentThrottle to currentThrottle - 0.002.
            lock throttle to currentThrottle.
        }
    } else if eta:apoapsis < 50 {
        if throttle < 1 {
            set currentThrottle to currentThrottle + 0.002.
            lock throttle to currentThrottle.
        }
    }
}

lock throttle to 0.
print "Gravity turn complete, waiting to be close to apoapsis to circularize...".

until ship:altitude >= 70000
{    
    lock steering to heading(90, 5).
    
    if ship:apoapsis < apoapsis_threshold
    {
        lock throttle to 0.2.
    }
    else
    {
        lock throttle to 0.
    }
}

lock throttle to 0.

set targetV to sqrt(ship:body:mu/(ship:orbit:body:radius + ship:orbit:apoapsis)). //this is the velocity that we need to be going at AP to be circular
set apVel to sqrt(((1 - ship:orbit:ECCENTRICITY) * ship:orbit:body:mu) / ((1 + ship:orbit:ECCENTRICITY) * ship:orbit:SEMIMAJORAXIS)). //this is how fast we will be going
set dv to targetV - apVel. // this is the deltaV
set mynode to node(time:seconds + eta:apoapsis, 0, 0, dv). // create a new maneuver node
add mynode. // add the node to our trajectory 

print mynode:TIME. // print the time of the node

set tset to 0.
lock throttle to tset.

set max_acc to ship:maxthrust / ship:mass.
set burn_duration to mynode:deltav:mag / max_acc.

print "Burn duration: " + round(burn_duration, 1) + "s".

kuniverse:timewarp:warpto(time:seconds + eta:apoapsis - burn_duration/2 - 60).
wait until kuniverse:timewarp:issettled.

lock steering to mynode:deltav.
wait until vAng(ship:facing:vector, mynode:deltav) < 1.

wait until mynode:eta <= (burn_duration/2).

set done to false.

set dv0 to mynode:deltav.

until done
{
    set max_acc to ship:maxthrust / ship:mass.
    set tset to min(mynode:deltav:mag/max_acc,1).

    if vdot(dv0, mynode:deltav) < 0
    {
        print "End burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s, vdot: " + round(vdot(dv0, mynode:deltav),1).
        lock throttle to 0.
        break.
    }

    if mynode:deltav:mag < 0.1
    {
        print "Finalizing burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s, vdot: " + round(vdot(dv0, mynode:deltav),1).
        wait until vdot(dv0, mynode:deltav) < 0.5.

        lock throttle to 0.
        set done to True.
    }
}

remove mynode.

// Final adjustments
lock steering to heading(90, 0).

print "Circularization complete.".