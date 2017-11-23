SET Runway_start_Latitude TO -0.048585977.
SET Runway_start_Longitude TO -74.72454.
SET Runway_start_Altitude TO 70.96860920.
SET Runway_end_Latitude TO -0.050211.
SET Runway_end_Longitude TO -74.4837820.
SET Runway_end_Altitude TO 70.50522.

SET Final_approch_Latitude TO 3*(Runway_start_Latitude-Runway_end_Latitude)+Runway_start_Latitude.
SET Final_approch_Longitude TO 3*(Runway_start_Longitude-Runway_end_Longitude)+Runway_start_Longitude.
SET Final_approch_Altitude TO 1000.

SET target_v TO 210.
SET Minimum_dynamic_pressure TO 0.03.
SET Maximum_dynamic_pressure TO 1.
SET low_altitude_warning TO 200.
SET Maximum_banking TO 60.
SET Maximum_side_slip TO 10.
SET Maximum_AoA TO 15.


// function returning the east vector of kerbin
FUNCTION east_for {
  parameter ves.

  return vcrs(ves:up:vector, ves:north:vector).
}


// function returning the roll angle of plane, negativ is left wing lower than right in chase cam
FUNCTION Roll_angle {
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
FUNCTION Pitch_angle {
  parameter ves.

  return 90 - vang(ves:up:vector, ves:facing:forevector).
}


// function returning pitch angle
FUNCTION Pitch_movement {
  parameter ves.

  return 90 - vang(ves:up:vector, ves:SRFPROGRADE:forevector).
}


// function returning compass course
FUNCTION Yaw_angle {
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
FUNCTION Yaw_movement {
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


// function returning angle of deveation between compass course and actuall course
FUNCTION Horizontal_slip {
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


// function returning angle of devation between Horizontal pointing and movment
FUNCTION Vertical_slip {
  parameter ves.

  return Pitch_angle(ves) - Pitch_movement(ves).
}


// function returning Yaw slip of plane
FUNCTION Yaw_slip {
  parameter ves.

  return Horizontal_slip(ves) * COS(Roll_angle(SHIP)) + Vertical_slip(ves) * SIN(Roll_angle(SHIP)).
}


// function returning AoA of plane
FUNCTION Pitch_slip {
  parameter ves.

  return Horizontal_slip(ves) * SIN(Roll_angle(SHIP)) + Vertical_slip(ves) * COS(Roll_angle(SHIP)).
}


// function returning total angle of slip of compass and horizontal slip
FUNCTION Total_slip {
  parameter ves.

  return SQRT(Horizontal_slip(ves)^2 + Vertical_slip(ves)^2).
}


// function transforming any n*360 degree angle to a range of [0,360]
FUNCTION to_smallest_degree {
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


// function setting roll to follow compass course
FUNCTION set_Roll {
  parameter Target_Latitude.
  parameter Target_Longitude.
  parameter banking.

  SET Diff_Latitude TO Target_Latitude - SHIP:LATITUDE.
  SET Diff_Longitude TO Target_Longitude-SHIP:LONGITUDE.
  SET new_course TO ARCTAN2(Diff_Longitude ,Diff_Latitude).
  SET new_course TO to_smallest_degree(new_course).
  SET course_diff TO new_course - Yaw_movement(SHIP).
  IF ABS(course_diff) < 10 {
    RETURN -banking/10*course_diff.
    }
  ELSE {
    IF course_diff < 0 {
     RETURN banking.
     }
    ELSE{
     RETURN -banking.
     }
    }
  }


FUNCTION set_Pitch {
  parameter Target_Altitude.
  parameter max_climb_angle.
  SET Diff_Altitude TO Target_Altitude - SHIP:ALTITUDE.
  IF ABS(Diff_Altitude/SHIP:AIRSPEED) < 1 {
    RETURN +max_climb_angle/SHIP:AIRSPEED / 1 * Diff_Altitude.
  }
  ELSE {
    IF Diff_Altitude < 0 {
     RETURN -max_climb_angle.
     }
    ELSE{
     RETURN +max_climb_angle.
     }
    }
}


// function returning the Latitude of a virtuall line continued from west to east side of runway, paramteter indicates the lenght of the line
// 0 is east side of runway, 1 one times the lenght of the runway in east direction...
FUNCTION Runway_Latitude_vector {
  Parameter lenght.

  RETURN lenght * (Runway_start_Latitude-Runway_end_Latitude)+Runway_start_Latitude.
}


// function returning the Longitude of a virtuall line continued from west to east side of runway, paramteter indicates the lenght of the line
// 0 is east side of runway, 1 one times the lenght of the runway in east direction...
FUNCTION Runway_Longitude_vector{
  Parameter lenght.

  RETURN lenght * (Runway_start_Longitude-Runway_end_Longitude)+Runway_start_Longitude.
}


// function returning distance to east side of runway , only concidering coordinates, not altitude
FUNCTION distance_to_runway {
  Parameter Latitude.
  Parameter Longitude.

  RETURN KERBIN:RADIUS*SQRT((Latitude-Runway_start_Latitude)^2 + (Longitude-Runway_start_LONGITUDE)^2).
}



SET PID_Thrust_Kp TO 0.1.
SET PID_Thrust_Ki TO 0.01.
SET PID_Thrust_Kd TO 0.5.
SET PID_Thrust TO PIDLOOP(PID_Thrust_Kp, PID_Thrust_Ki, PID_Thrust_Kd).
SET PID_Thrust:MAXOUTPUT TO 1.
SET PID_Thrust:MINOUTPUT TO 0.
SET PID_Thrust:SETPOINT TO 0.

SET PID_Yaw_Kp TO 0.005.
SET PID_Yaw_Ki TO 0.0005.
SET PID_Yaw_Kd TO 0.02.
SET PID_Yaw TO PIDLOOP(PID_Yaw_Kp , PID_Yaw_Ki, PID_Yaw_Kd).
SET PID_Yaw:MAXOUTPUT TO 1.
SET PID_Yaw:MINOUTPUT TO -1.
SET PID_Yaw:SETPOINT TO 0.

SET PID_Pitch_Kp TO 0.001.
SET PID_Pitch_Ki TO 0.003.
SET PID_Pitch_Kd TO 0.002.
SET PID_Pitch TO PIDLOOP(PID_Pitch_Kp, PID_Pitch_Ki, PID_Pitch_Kd).
SET PID_Pitch:MAXOUTPUT TO 1.
SET PID_Pitch:MINOUTPUT TO -1.
SET PID_Pitch:SETPOINT TO 0.

SET PID_Roll_Kp TO 0.001.
SET PID_Roll_Ki TO 0.0001.
SET PID_Roll_Kd TO 0.005.
SET PID_Roll TO PIDLOOP(PID_Roll_Kp , PID_Roll_Ki , PID_Roll_Kd).
SET PID_Roll:MAXOUTPUT TO 1.
SET PID_Roll:MINOUTPUT TO -1.
SET PID_Roll:SETPOINT TO 0.



SET ref_t TO TIME:SECONDS().


SET ROUTINE TO LIST("Flight", "Final_approach", "Touchdown", "Rolling").
SET State TO "OK".
SET i TO 0.
FUNCTION Controller {
  PRINT State.
  IF State {
    IF ROUTINE[i] = "Flight"{
      // SET PID_Pitch:KD TO 0.1 + 1*(TIME:SECONDS() - ref_t).
      SET Speed_expression TO SHIP:AIRSPEED-target_v.
      SET Yaw_Expression TO -Yaw_slip(SHIP).
      SET Pitch_Expression TO -set_Pitch(1070, 15) + Pitch_angle(SHIP) - 5.
      SET Roll_Expression TO set_Roll(Runway_Latitude_vector(3), Runway_Longitude_vector(3), 45) + Roll_angle(SHIP).

      IF ABS(Runway_Latitude_vector(3)-SHIP:LATITUDE)<0.01 AND abs(Runway_Longitude_vector(3)-SHIP:LONGITUDE)< 0.01 {
        SET i TO i + 1.
      }
    }

    IF ROUTINE[i] = "Final_approach"{
      SET Speed_expression TO SHIP:AIRSPEED-target_v + 80*(1-distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(3), Runway_Longitude_vector(3))).
      SET Yaw_Expression TO -1*(-Yaw_slip(SHIP) + SIN(Roll_angle(SHIP))*Total_slip(SHIP)).
      SET Pitch_Expression TO -set_Pitch(+1070 - 970 * (1 - distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(3), Runway_Longitude_vector(3))), 15) + Pitch_angle(SHIP).
      SET Roll_Expression TO set_Roll(Runway_Latitude_vector(-0.3+distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(1), Runway_Longitude_vector(1))),
      Runway_longitude_vector(-0.3+distance_to_runway(SHIP:LATITUDE, SHIP:LONGITUDE)/distance_to_runway(Runway_Latitude_vector(1), Runway_Longitude_vector(1))), 30) + Roll_angle(SHIP).

      IF ABS(Runway_Latitude_vector(0)-SHIP:LATITUDE)<0.01 AND abs(Runway_Longitude_vector(0)-SHIP:LONGITUDE)< 0.01 {
        SET i TO i + 1.
      }
    }

    IF ROUTINE[i] = "Touchdown"{
      SET Speed_expression TO SHIP:AIRSPEED - 120.
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

  ELSE IF State = "high_AoA" and ROUTINE[i] = "Flight"{
    SET Pitch_Expression TO -1*(-Pitch_slip(SHIP) + COS(Roll_angle(SHIP))*Total_slip(SHIP)).
    PRINT "high AoA".
  }


}


FUNCTION Status_Check{
  IF SHIP:DYNAMICPRESSURE < Minimum_dynamic_pressure {
    SET State TO "low_airspeed".
  }
  ELSE IF ALT:RADAR < low_altitude_warning {
    SET State TO "low_altitude".
  }
  ELSE IF SHIP:VERTICALSPEED/ALT:RADAR > 20 {
    SET State TO "high_sinkrate".
  }
  ELSE IF SHIP:DYNAMICPRESSURE > Maximum_dynamic_pressure {
    SET State TO "high_airspeed".
  }
  ELSE IF Roll_angle(SHIP) > Maximum_banking {
    SET State TO "high_banking".
  }
  ELSE IF SIN(Roll_angle(SHIP))*Total_slip(SHIP) > Maximum_side_slip {
    SET State TO "high_side_slip".
  }
  ELSE IF COS(Roll_angle(SHIP))*Total_slip(SHIP) > Maximum_AoA {
    SET State TO "high_AoA".
  }

  ELSE {
    SET State TO "OK".
  }
}


UNTIL FALSE{
  Status_check().
  Controller().
  SET THROTTLE to PID_Thrust:UPDATE(TIME:SECONDS, Speed_expression).
  SET SHIP:CONTROL:YAW TO PID_Yaw:UPDATE(TIME:SECONDS, Yaw_Expression).
  SET SHIP:CONTROL:PITCH TO PID_Pitch:UPDATE(TIME:SECONDS, Pitch_Expression).
  SET SHIP:CONTROL:ROLL TO PID_Roll:UPDATE(TIME:SECONDS, Roll_Expression).
  WAIT 0.01.
}
