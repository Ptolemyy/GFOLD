// raw_control pitch swing test
// Purpose: hold full pitch input with fixed low throttle for oscillation observation.

UNLOCK THROTTLE.
UNLOCK STEERING.

SAS OFF.
RCS OFF.

SET TARGET_THR TO 0.2.
SET TARGET_PITCH TO 1.0.

PRINT "raw_control pitch test running".
PRINT "RCS = OFF, throttle = 0.2, pitch = full.".
PRINT "Stop with Ctrl+C when done.".

UNTIL FALSE {
  // Direct flight-control channel for pitch/yaw/roll input.
  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO TARGET_THR.
  SET SHIP:CONTROL:PITCH TO TARGET_PITCH.
  SET SHIP:CONTROL:YAW TO 0.
  SET SHIP:CONTROL:ROLL TO 0.
  WAIT 0.02.
}.
