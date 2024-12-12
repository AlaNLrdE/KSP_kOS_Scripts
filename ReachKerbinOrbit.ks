// Initialize variables
set pitch to 90. // Initial pitch angle
set speed_threshold to 100. // Speed threshold to start changing pitch
set apoapsis_threshold to 80000. // Apoapsis height threshold to change pitch to 0
set altitude_threshold to 10000. // Altitude threshold to complete pitch change to 45 degrees
set gravity_turn_end to 10000. // End gravity turn at 10 km

// Initialize PID controller variables for smoother error correction
set Kp to 0.0005. // Decreased proportional gain
set Ki to 0.0002. // Increased integral gain
set Kd to 0.0002. // Decreased derivative gain
set integral to 0.
set previous_error to 0.
// Adjust throttle based on time to apoapsis using PID controller
set setpoint to 50. // Desired time to apoapsis
set pid_output to 1. // Initial PID output

// Define stage properties directly
set lockInitialAltitude to false.

// Define PID controller function
function PID {
    parameter l_setpoint, l_measured_value, l_Kp, l_Ki, l_Kd, l_integral, l_previous_error.
    set l_error to l_setpoint - l_measured_value.
    set l_integral to l_integral + l_error * time:seconds.
    set l_derivative to (l_error - l_previous_error) / time:seconds.
    set output to l_Kp * l_error + l_Ki * l_integral + l_Kd * l_derivative.
    set previous_error to l_error.
    return output.
}

// Define gravity turn function
function bxbGravityTurn {
    // Launch the rocket
    lock throttle to 1. // Set throttle to maximum
    print "(State 1) Starting gravity turn...".

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
            // Apply the PID output to throttle
            lock throttle to pid_output.
            print "(2) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
        } else if altitude >= altitude_threshold and ship:apoapsis < apoapsis_threshold {
            // Adjust pitch based on altitude
            if ship:altitude > 40000 {
                set pitch to 20.
            } else {
                set pitch to 45.
            }
            lock steering to heading(90, pitch).
            // Apply the PID output to throttle
            lock throttle to pid_output.
            print "(3) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
        } else {
            lock steering to heading(90, 90).
            // Apply the PID output to throttle
            lock throttle to 1.
            print "(1) Time: " + round(missiontime, 1) + "s, Altitude: " + round(altitude, 1) + "m, Velocity: " + round(ship:velocity:surface:mag, 1) + "m/s, Pitch: " + round(pitch, 1) + "°".
        }

        set measured_value to eta:apoapsis.
        set pid_output to PID(setpoint, measured_value, Kp, Ki, Kd, integral, previous_error).

        // Clamp the PID output to throttle limits
        set pid_output to max(0, min(1, pid_output)).

        if pid_output < 0.15 {
            set pid_output to 0.15.
        }

        wait 0.1.
    }

    // Cut off the throttle
    lock throttle to 0.
    print "Gravity turn complete, reaching required altitude to circularize...".
}

// Define circularization node creation function
function bxbCreateCircularizationNode {
    print "Creating circularization node...".
    // Calculate the required deltaV for circularization
    set targetV to sqrt(ship:body:mu/(ship:orbit:body:radius + ship:orbit:apoapsis)).
    set apVel to sqrt(((1 - ship:orbit:ECCENTRICITY) * ship:orbit:body:mu) / ((1 + ship:orbit:ECCENTRICITY) * ship:orbit:SEMIMAJORAXIS)).
    set dv to targetV - apVel.

    // Create a new maneuver node for circularization
    set mynode to node(time:seconds + eta:apoapsis, 0, 0, dv).
    add mynode.

    print "Node time: " + mynode:TIME. // Print the time of the node
    print "Circularization node created.".
}

// Define ascending outsise atmosphere and compensating for drag function
function bxbAscendingOutsideAtmosphere {
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
}

// Define maneuver node execution function
function bxbExecuteManeuverNode {
    print "Executing maneuver node...".
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
    print "Aligning with the maneuver node...".
    lock steering to mynode:deltav.
    wait until vAng(ship:facing:vector, mynode:deltav) < 1.
    print "Alignment complete.".

    // Wait until it's time to start the burn
    print "Waiting to " + burn_duration/2 +" second mark before starting maneuver burn".
    wait until mynode:eta <= (burn_duration/2).
    print "Starting burn...".

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
    print "Maneuver node executed.".
}

// Main loop
print "Starting ascent to Kerbin orbit...".
// Perform gravity turn - State 1
bxbGravityTurn(). 
// Ascend to apoapsis - State 2
bxbAscendingOutsideAtmosphere(). 
// Create maneuver node - State 3
bxbCreateCircularizationNode().
// Maneauver to circularize orbit execution - State 4
bxbExecuteManeuverNode().

print "Circularization complete.".