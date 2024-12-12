// Initialize variables
set pitch to 90. // Initial pitch angle
set speed_threshold to 100. // Speed threshold to start changing pitch
set apoapsis_threshold to 80000. // Apoapsis height threshold to change pitch to 0
set altitude_threshold to 10000. // Altitude threshold to complete pitch change to 45 degrees
set gravity_turn_end to 10000. // End gravity turn at 10 km

// Define stage properties directly
set lockInitialAltitude to false.

// Launch the rocket
lock throttle to 1. // Set throttle to maximum
stage. // Activate the first stage

set currentThrottle to throttle.

// Main loop
print "Starting gravity turn...".

// Burn until apoapsis reaches the threshold
until ship:apoapsis >= apoapsis_threshold {
    // Update pitch based on speed and apoapsis height
    if ship:velocity:surface:mag > speed_threshold and ship:apoapsis < apoapsis_threshold and altitude < altitude_threshold {
        if not lockInitialAltitude {
            set lockInitialAltitude to true.
            set initialAltitude to altitude.
        }
        // Calculate pitch based on altitude
        set pitch to 90 - (altitude - initialAltitude) / (gravity_turn_end - initialAltitude) * 45.
        lock steering to heading(90, pitch).
        print "(2) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
    } else if altitude >= altitude_threshold and ship:apoapsis < apoapsis_threshold {
        // Adjust pitch based on altitude
        if ship:altitude > 40000 {
            set pitch to 15.
        } else {
            set pitch to 45.
        }
        lock steering to heading(90, pitch).
        print "(3) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
    } else {
        lock steering to heading(90, 90).
        print "(1) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
    }

    // Adjust throttle based on time to apoapsis
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

// Cut off the throttle
lock throttle to 0.
print "Gravity turn complete, reaching required altitude to circularize...".

// Wait until the ship reaches 70 km altitude
until ship:altitude >= 70000 {
    lock steering to heading(90, 5).
    
    // Adjust throttle based on apoapsis height
    if ship:apoapsis < apoapsis_threshold {
        lock throttle to 0.2.
    } else {
        lock throttle to 0.
    }
}

// Cut off the throttle
lock throttle to 0.

// Calculate the required deltaV for circularization
set targetV to sqrt(ship:body:mu/(ship:orbit:body:radius + ship:orbit:apoapsis)).
set apVel to sqrt(((1 - ship:orbit:ECCENTRICITY) * ship:orbit:body:mu) / ((1 + ship:orbit:ECCENTRICITY) * ship:orbit:SEMIMAJORAXIS)).
set dv to targetV - apVel.

// Create a new maneuver node for circularization
set mynode to node(time:seconds + eta:apoapsis, 0, 0, dv).
add mynode.

print mynode:TIME. // Print the time of the node

// Initialize throttle and calculate burn duration
set tset to 0.
lock throttle to tset.

set max_acc to ship:maxthrust / ship:mass.
set burn_duration to mynode:deltav:mag / max_acc.

print "Burn duration: " + round(burn_duration, 1) + "s".

// Warp to the burn start time
kuniverse:timewarp:warpto(time:seconds + eta:apoapsis - burn_duration/2 - 60).
wait until kuniverse:timewarp:issettled.

// Align with the maneuver node
lock steering to mynode:deltav.
wait until vAng(ship:facing:vector, mynode:deltav) < 1.

// Wait until it's time to start the burn
wait until mynode:eta <= (burn_duration/2).

set done to false.
set dv0 to mynode:deltav.

// Execute the burn
until done {
    set max_acc to ship:maxthrust / ship:mass.
    set tset to min(mynode:deltav:mag/max_acc,1).

    if vdot(dv0, mynode:deltav) < 0 {
        print "End burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s, vdot: " + round(vdot(dv0, mynode:deltav),1).
        lock throttle to 0.
        break.
    }

    if mynode:deltav:mag < 0.1 {
        print "Finalizing burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s, vdot: " + round(vdot(dv0, mynode:deltav),1).
        wait until vdot(dv0, mynode:deltav) < 0.5.

        lock throttle to 0.
        set done to True.
    }
}

// Remove the maneuver node
remove mynode.

// Final adjustments
lock steering to heading(90, 0).

print "Circularization complete.".