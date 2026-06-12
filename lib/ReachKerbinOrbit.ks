// Flight configuration and state variables
global pitch is 90.                  // Initial pitch angle
global speed_threshold is 100.       // Speed threshold to start changing pitch
global apoapsis_threshold is 80000.  // Target apoapsis height (will be set by user)

// PID controller configuration (throttle based on time to apoapsis)
global Kp is 0.0005.
global Ki is 0.0002.
global Kd is 0.0002.
global integral is 0.
global previous_error is 0.
global setpoint is 50.               // Desired time to apoapsis (seconds)
global pid_output is 1.              // Initial PID output

// Flight logic control variables
global fairings_deployed is false.

// Automatically stage if engine flameout or no thrust is detected
function bxbCheckAutostage {
    if ship:maxthrust = 0 {
        print "Flameout or lack of thrust detected. Separating stage...".
        stage.
        wait 1.5. // Wait for stage separation and engine ignition
    }
}

// Automatically deploy fairings when altitude is above 50km
function bxbCheckFairings {
    if not fairings_deployed and altitude > 50000 {
        print "Altitude above 50 km. Deploying fairings...".
        for p in ship:parts {
            for m in p:modules {
                if m = "ModuleProceduralFairing" or m = "ModuleSimpleAdjustableFairings" {
                    local part_mod is p:getmodule(m).
                    if part_mod:hasEvent("deploy") {
                        part_mod:doEvent("deploy").
                    } else if part_mod:hasEvent("jettison") {
                        part_mod:doEvent("jettison").
                    }
                }
            }
        }
        set fairings_deployed to true.
    }
}

// Calculate the recommended pitch angle based on current altitude (matches standard KSP gravity turn profiles)
function bxbCalculatePitch {
    parameter current_alt.
    
    // Hold vertical flight if speed is too low for aerodynamic control
    if ship:velocity:surface:mag < speed_threshold {
        return 90.
    }
    
    // 0-1 km: 90 degrees (vertical ascent)
    if current_alt < 1000 {
        return 90.
    }
    
    // 1-30 km: Smooth transition from 90 to 30 degrees (slow descent)
    if current_alt < 30000 {
        return 90 - 60 * (current_alt - 1000) / 29000.
    }
    
    // 30-70 km: Smooth transition from 30 to 0 degrees (horizontal alignment)
    if current_alt < 70000 {
        return 30 - 30 * (current_alt - 30000) / 40000.
    }
    
    return 0.
}

// Read numeric input from the terminal with a default value fallback
function bxbGetNumberInput {
    parameter default_val.
    local input_str is "".
    until false {
        local ch is terminal:input:getchar().
        if ch = terminal:input:enter {
            break.
        } else if ch = terminal:input:backspace {
            if input_str:length > 0 {
                set input_str to input_str:substring(0, input_str:length - 1).
                print ch.
                print " ".
                print ch.
            }
        } else {
            // Accept only digits and decimal point
            if ch = "0" or ch = "1" or ch = "2" or ch = "3" or ch = "4" or ch = "5" or ch = "6" or ch = "7" or ch = "8" or ch = "9" or ch = "." {
                set input_str to input_str + ch.
                print ch.
            }
        }
    }
    print "".
    if input_str = "" or input_str = "." {
        return default_val.
    }
    return input_str:toscalar(default_val).
}

// PID controller function
function PID {
    parameter l_setpoint, l_measured_value, l_Kp, l_Ki, l_Kd, l_integral, l_previous_error.
    local l_error is l_setpoint - l_measured_value.
    set l_integral to l_integral + l_error * time:seconds.
    local l_derivative is (l_error - l_previous_error) / time:seconds.
    local output is l_Kp * l_error + l_Ki * l_integral + l_Kd * l_derivative.
    set previous_error to l_error.
    return output.
}

// Gravity turn maneuver execution (State 1)
function bxbGravityTurn {
    lock throttle to 1. // Full throttle initially
    lock steering to heading(90, pitch).

    until ship:apoapsis >= apoapsis_threshold {
        bxbCheckAutostage().
        bxbCheckFairings().

        // Dynamically compute pitch based on altitude
        set pitch to bxbCalculatePitch(altitude).

        bxbDisplayMissionInfo("|----- Gravity Turn / State 1 -----|").

        local measured_value is eta:apoapsis.
        set pid_output to PID(setpoint, measured_value, Kp, Ki, Kd, integral, previous_error).

        // Clamp PID output between safe throttle limits (0.15 to 1.0)
        set pid_output to max(0.15, min(1, pid_output)).
        lock throttle to pid_output.

        wait 0.1.
    }

    lock throttle to 0.
}

// Ascend outside atmosphere and compensate for drag (State 2)
function bxbAscendingOutsideAtmosphere {
    local atmosphere_height is 70000. // Kerbin atmosphere limit in meters
    until ship:altitude >= atmosphere_height {
        bxbCheckAutostage().
        bxbCheckFairings().
        lock steering to heading(90, 0). // Point straight to the horizon (pitch 0)
        bxbDisplayMissionInfo("|----- Ascending above atmosphere / State 2 -----|").
        
        if ship:apoapsis < apoapsis_threshold {
            lock throttle to 0.2.
        } else {
            lock throttle to 0.
        }
        wait 0.1.
    }
    lock throttle to 0.
}

// Calculate and create circularization maneuver node (State 3)
function bxbCreateCircularizationNode {
    // Calculate orbital insertion requirements at apoapsis
    local targetV is sqrt(ship:body:mu / (ship:orbit:body:radius + ship:orbit:apoapsis)).
    local apVel is sqrt(((1 - ship:orbit:ECCENTRICITY) * ship:orbit:body:mu) / ((1 + ship:orbit:ECCENTRICITY) * ship:orbit:SEMIMAJORAXIS)).
    local dv is targetV - apVel.

    // Create and schedule the maneuver node
    global mynode is node(time:seconds + eta:apoapsis, 0, 0, dv).
    add mynode.
    bxbDisplayMissionInfo("|----- Circularization Node Creation / State 3 -----|").
    print "Node time: " + round(mynode:TIME, 1).
    print "Circularization node created.".
}

// Execute the maneuver node (State 4)
function bxbExecuteManeuverNode {
    bxbDisplayMissionInfo("|----- Executing maneuver node / State 4 -----|").
    
    local tset is 0.
    lock throttle to tset.

    local max_acc is ship:maxthrust / ship:mass.
    local burn_duration is mynode:deltav:mag / max_acc.

    print "Burn duration: " + round(burn_duration, 1) + "s".

    // Align ship to node delta V vector BEFORE warp
    print "Aligning with the maneuver node...".
    rcs on. // Enable RCS to assist alignment
    lock steering to mynode:deltav.
    wait until vAng(ship:facing:vector, mynode:deltav) < 1.
    print "Alignment complete.".

    // Warp to node minus alignment and buffer times
    print "Warping to burn start...".
    kuniverse:timewarp:warpto(time:seconds + eta:apoapsis - burn_duration / 2 - 60).
    wait until kuniverse:timewarp:issettled.

    // Wait for the exact start window
    print "Waiting to " + round(burn_duration / 2, 1) + "s mark before starting burn".
    wait until mynode:eta <= (burn_duration / 2).
    print "Starting burn...".

    local done is false.
    local dv0 is mynode:deltav.

    until done {
        bxbCheckAutostage().
        set max_acc to ship:maxthrust / ship:mass.
        set tset to min(mynode:deltav:mag / max_acc, 1).

        // If we overshoot (vector dot product goes negative), stop burn
        if vdot(dv0, mynode:deltav) < 0 {
            bxbDisplayMissionInfo("|----- Executing maneuver node / State 4 -----|").
            print "End burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s".
            lock throttle to 0.
            break.
        }

        // Finalize high precision low thrust cutoff
        if mynode:deltav:mag < 0.1 {
            bxbDisplayMissionInfo("|----- Executing maneuver node / State 4 -----|").
            print "Finalizing burn, remain dv " + round(mynode:deltav:mag, 1) + "m/s".
            wait until vdot(dv0, mynode:deltav) < 0.5.
            lock throttle to 0.
            set done to true.
        }
        wait 0.05.
    }

    remove mynode.
    rcs off. // Disable RCS after burn is complete to save monopropellant
    lock steering to heading(90, 0).
    print "Maneuver node executed.".

    // Deploy solar panels, radiators, and antennas
    print "Deploying solar panels and antennas...".
    panels on.
    for p in ship:parts {
        for m in p:modules {
            if m = "ModuleDeployableAntenna" or m = "ModuleRealAntenna" or m = "ModuleAnimateGeneric" {
                local part_mod is p:getmodule(m).
                for ev in part_mod:allEventNames {
                    if ev:contains("extend") or ev:contains("deploy") {
                        part_mod:doEvent(ev).
                    }
                }
            }
        }
    }
}

// Render telemetries and state info on screen
function bxbDisplayMissionInfo {
    parameter l_customMessage is "General mission information: ".
    clearscreen.
    print l_customMessage.
    print "-----------------------------------------".
    print "Time:             " + round(missiontime, 1) + " s.".
    print "Altitude:         " + round(altitude, 0) + " m / Target: " + round(apoapsis_threshold, 0) + " m.".
    print "Velocity:         " + round(ship:velocity:surface:mag, 1) + " m/s.".
    print "Pitch:            " + round(pitch, 1) + "°".
    print "Time to apoapsis: " + round(eta:apoapsis, 1) + " s.".
    print "Throttle:         " + round(throttle * 100, 1) + "%".
}

// Entry point function
global function bxbReachKerbinOrbit {
    clearscreen.
    print "=========================================".
    print "       KSP kOS Orbital Ascent Controller".
    print "=========================================".
    print "Enter the desired orbit altitude (km):".
    print "(Press ENTER to use default value: 80 km)".
    print "Altitude > ".
    
    local target_km is bxbGetNumberInput(80).
    set apoapsis_threshold to target_km * 1000.
    
    print " ".
    print "Configured to orbit at: " + target_km + " km (" + apoapsis_threshold + " m)".
    print "Starting launch sequence in 3 seconds...".
    wait 3.

    print "Starting ascent to Kerbin orbit...".
    bxbGravityTurn().                  // State 1
    bxbAscendingOutsideAtmosphere().   // State 2
    bxbCreateCircularizationNode().    // State 3
    bxbExecuteManeuverNode().          // State 4

    print "Circularization complete.".
}