// function returning the east vector of kerbin
FUNCTION east_for {
  parameter ves.

  return vcrs(ves:up:vector, ves:north:vector).
}


// function returning the roll angle of plane, negativ is left wing lower than right in chase cam
FUNCTION Roll_angle{
parameter ves.

  if vang(ship:facing:vector,ship:up:vector) < 0.2 { //this is the dead zone for roll when the ship is vertical
    return 0.
  } else {
    local raw is vang(vxcl(ship:facing:vector,ship:up:vector), ves:facing:starvector).
    if vang(ves:up:vector, ves:facing:topvector) > 90 {
      if raw > 90 {
        return 270 - raw.
      } else {
        return -90 - raw.
      }
    } else {
      return raw - 90.}
      }
}


// function returning pitch angle
FUNCTION Pitch_angle{
  parameter ves.

  return 90 - vang(ves:up:vector, ves:facing:forevector).
}


// function returning pitch angle
FUNCTION Pitch_movement{
  parameter ves.

  return 90 - vang(ves:up:vector, ves:SRFPROGRADE:forevector).
}


// function returning compass course
FUNCTION Yaw_angle{
  parameter ves.

  local pointing is ves:facing:forevector.
  local east is east_for(ves).

  local trig_x is vdot(ves:north:vector, pointing).
  local trig_y is vdot(east, pointing).

  local result is arctan2(trig_y, trig_x).

  if result < 0 {
    return 360 + result.
  } else {
    return result.
  }
}


// function returning actual compass course based on movement
FUNCTION Yaw_movement{
  parameter ves.

  local pointing is ves:SRFPROGRADE:forevector.
  local east is east_for(ves).

  local trig_x is vdot(ves:north:vector, pointing).
  local trig_y is vdot(east, pointing).

  local result is arctan2(trig_y, trig_x).

  if result < 0 {
    return 360 + result.
  } else {
    return result.
  }
}


FUNCTION Yaw_slip{
  parameter ves.
  IF ABS(Yaw_angle(ves) - Yaw_movement(ves)) < 180 {
    RETURN Yaw_angle(ves) - Yaw_movement(ves).
    }
  ELSE {
    IF Yaw_angle(ves) -  Yaw_movement(ves) < 0 {
      RETURN 360 - Yaw_movement(ves) + Yaw_angle(ves).
      }
    ELSE {
      RETURN - 360 - Yaw_movement(ves) + Yaw_angle(ves).
    }
    }
  }


FUNCTION Pitch_slip{
  parameter ves.
  return Pitch_angle(ves) - Pitch_movement(ves).
}


FUNCTION Total_slip{
  parameter ves.
  return SQRT(Yaw_slip(ves)^2 + Pitch_slip(ves)^2).
}



FUNCTION to_smallest_degree{
  parameter degree.
  IF degree < 0 {
  UNTIL degree > 0 {
  SET degree TO degree + 360.
  }
  RETURN degree.
  }
  ELSE {
  UNTIL degree < 360 {
  SET degree TO degree - 360.
  }
  RETURN degree.
  }
}


FUNCTION set_Roll{
  parameter Target_Latitude.
  parameter Target_Longitude.

  SET Diff_Latitude TO Target_Latitude-SHIP:LATITUDE.
  SET Diff_Longitude TO Target_Longitude-SHIP:LONGITUDE.
  SET new_course TO ARCTAN2(Diff_Longitude ,Diff_Latitude).
  SET new_course TO to_smallest_degree(new_course).
  SET course_diff TO new_course - Yaw_movement(SHIP).
  PRINT new_course.
  IF ABS(course_diff) < 10 {
    RETURN -6*course_diff.
    }
  ELSE {
    IF course_diff < 0{
     RETURN 60.
     }
    ELSE{
     RETURN -60.
     }
    }
  }








SET Runway_start_Latitude TO -0.048585977.
SET Runway_start_Longitude TO -74.72454.
SET Runway_start_Altitude TO 70.96860920.
SET Runway_end_Latitude TO -0.050211.
SET Runway_end_Longitude TO -74.4837820.
SET Runway_end_Altitude TO 70.50522.

SET Final_approch_Latitude TO 3*(Runway_start_Latitude-Runway_end_Latitude)+Runway_start_Latitude.
SET Final_approch_Longitude TO 3*(Runway_start_Longitude-Runway_end_Longitude)+Runway_start_Longitude.
SET Final_approch_Altitude TO 1000.


FUNCTION Runway_Latitude_vector{
Parameter lenght.
RETURN lenght * (Runway_start_Latitude-Runway_end_Latitude)+Runway_start_Latitude.
}


FUNCTION Runway_Longitude_vector{
Parameter lenght.
RETURN lenght * (Runway_start_Longitude-Runway_end_Longitude)+Runway_start_Longitude.
}


FUNCTION distance_to_runway{
Parameter Latitude.
Parameter Longitude.

RETURN KERBIN:RADIUS*SQRT((Latitude-Runway_start_Latitude)^2 + (Longitude-Runway_start_LONGITUDE)^2).
}


SET target_v TO 150.
SET Minimum_dynamic_pressure TO 0.03.
SET i TO 0.


SET PID_Thrust TO PIDLOOP(0.1, 0.01, 0.5).
SET PID_Yaw TO PIDLOOP(0.01, 0.001, 0.05).
SET PID_Pitch TO PIDLOOP(0.01, 0.001, 0.05).
SET PID_Roll TO PIDLOOP(0.002, 0.0002, 0.01).

SET PID_Thrust:SETPOINT TO 0.
SET PID_Yaw:SETPOINT TO 0.
SET PID_Pitch:SETPOINT TO 0.
SET PID_Roll:SETPOINT TO 0.

SET ref_t TO TIME:SECONDS().


SET ROUTINE TO LIST("Flight", "Final_approach", "Touchdown", "Rolling").
SET Status TO "OK".


FUNCTION Controller{

IF ROUTINE[i] = "Flight"{
SET Speed_expression TO SHIP:AIRSPEED-target_v.
SET Yaw_Expression TO -1*(-Yaw_slip(SHIP) + SIN(Roll_angle(SHIP))*Total_slip(SHIP)).
SET Pitch_Expression TO SHIP:ALTITUDE-1070.
SET Roll_Expression TO set_Roll(Runway_Latitude_vector(3), Runway_Longitude_vector(3)) + Roll_angle(SHIP).

IF ABS(Runway_Latitude_vector(3)-SHIP:LATITUDE)<0.01 AND abs(Runway_Longitude_vector(3)-SHIP:LONGITUDE)< 0.01 {
SET i TO i + 1.
}
}

IF ROUTINE[i] = "Final_approach"{
SET Speed_expression TO SHIP:AIRSPEED-target_v + 80*(1-distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(3), Runway_Longitude_vector(3))).
SET Yaw_Expression TO -1*(-Yaw_slip(SHIP) + SIN(Roll_angle(SHIP))*Total_slip(SHIP)).
SET Pitch_Expression TO SHIP:ALTITUDE-1070 + 970 * (1 - distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(3), Runway_Longitude_vector(3))).
SET Roll_Expression TO set_Roll(Runway_Latitude_vector(-0.3+distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(1), Runway_Longitude_vector(1))),
 Runway_longitude_vector(-0.3+distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(1), Runway_Longitude_vector(1)))) + Roll_angle(SHIP).

IF ABS(Runway_Latitude_vector(0)-SHIP:LATITUDE)<0.01 AND abs(Runway_Longitude_vector(0)-SHIP:LONGITUDE)< 0.01 {
SET i TO i + 1.
}
}
IF ROUTINE[i] = "Touchdown"{
SET Speed_expression TO SHIP:AIRSPEED - 80.
SET Yaw_Expression TO -1*(-Yaw_slip(SHIP) + SIN(Roll_angle(SHIP))*Total_slip(SHIP)).
SET Pitch_Expression TO SHIP:VERTICALSPEED + 0.1.
SET Roll_Expression TO Roll_angle(SHIP).

IF SHIP:STATUS = "LANDED"{
SET i TO i + 1.
}
}

IF ROUTINE[i] = "Rolling"{
SET Speed_expression TO 1.
SET Yaw_Expression TO 0.
SET Pitch_Expression TO 0.
SET Roll_Expression TO 0.
}
}


FUNCTION Status_Check{
IF SHIP:DYNAMICPRESSURE < Minimum_dynamic_pressure {
  SET Status TO "low_airspeed".
}
ELSE {
  SET Status TO "OK".
}
}


UNTIL FALSE{
Status_check().
Controller().
SET THROTTLE to PID_Thrust:UPDATE(TIME:SECONDS, Speed_expression).
SET SHIP:CONTROL:YAW TO PID_Yaw:UPDATE(TIME:SECONDS, Yaw_Expression).
SET SHIP:CONTROL:PITCH TO PID_Pitch:UPDATE(TIME:SECONDS, Pitch_Expression).
SET SHIP:CONTROL:ROLL TO PID_Roll:UPDATE(TIME:SECONDS, Roll_Expression).
}
