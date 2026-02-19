// ===== Defaults (can be overridden by config.txt via RUNPATH) =====
SET LAT0 TO -0.0972.
SET LON0 TO -74.5577.

SET THROT1 TO 0.2.        // min throttle fraction
SET THROT2 TO 0.8.        // max throttle fraction
SET THETA_DEG TO 45.      // thrust cone half-angle (deg)
SET YGS_DEG TO 30.        // glide slope angle (deg)
SET UP_BIAS_M TO -3.       // altitude bias added to reported UP_M in mode0/mode1
SET CUTDOWN_ALTITUDE TO 2.0. // hard engine cutoff altitude using raw UP_M (no bias)
SET RECOMPUTE_ENABLED TO 1. // set to 1 to enable recompute trigger
SET RECOMPUTE_TIME TO 1.5.  // seconds between recompute triggers
SET LAST_RECOMPUTE_TIME TO 0.
SET RESEND_TIME TO 0.1.      // resend state interval when waiting for reply
SET DEPLOY_GEAR_TIME TO 8.0. // deploy gear when remain_tf <= this value

// ===== Optional config override =====
// Provide a config.txt in the same directory with lines like:
//   SET LAT0 TO -0.0972.
//   SET LON0 TO -74.5577.  LAUNCHPAD
//   SET ALT0 TO 0.
//   SET THROT1 TO 0.2.
//   SET THROT2 TO 0.8.
//   SET THETA_DEG TO 45.
//   SET YGS_DEG TO 30.

//   SET LAT0 TO -0.09694444.
//   SET LON0 TO -74.617. VAB
SET CONFIG_FILE TO "config.txt".
IF EXISTS(CONFIG_FILE) { RUNPATH(CONFIG_FILE). }.

// ===== Comm files =====
// Communication contract:
//   - We write state to send.txt (mode0 or mode1 config).
//   - PC solver writes results to receive.txt.
// receive.txt can contain:
//   (A) Request-only: "COMPUTE_FINISH,1" -> ask us to send mode1 config.
//   (B) Full profile: COMPUTE_FINISH + many U lines (up,north,east,t_abs) -> update U_LIST.
//   (C) "COMPUTE_FINISH,2" -> infeasible/no usable profile, disable recompute.
//   (D) "REMAIN_TF,<seconds>" -> remaining trajectory time from CPP.
// Explicitly use Archive volume paths to match PC-side writes.
SET SEND_FILE TO "0:/send.txt".
SET RECV_FILE TO "0:/receive.txt".
SET U_LIST TO LIST().        // each entry: [u_up,u_north,u_east,t_abs]
SET CR TO CHAR(13).
SET LF TO CHAR(10).
// Fixed loop time step for pacing; elapsed time uses TIME:SECONDS.
// ELAPSED_SEC is the single time axis for U scheduling and recompute.
SET LOOP_DT TO 0.02.
SET TIME0_SEC TO TIME:SECONDS.
SET ELAPSED_SEC TO 0.        // global time since script start
SET LOOP_TIME TO 0.          // main_tick loop delta
SET SEND_STATE_TIME TO TIME:SECONDS.
SET LOOP_INDEX TO 0.
SET SPEED TO 0.
SET U_MAG TO 0.
SET LAST_THR_CMD TO 0.0.       // last throttle command (0..1)
SET STEER_CMD TO PFRAME_TO_XYZ(V(0,0,1), GET_LH_ENU_AXES(BODY:POSITION:NORMALIZED)). // last steering command vector
SET RECOMPUTE_PENDING TO 0.  // recompute requested but not answered yet
SET HAS_U_LIST TO 0.         // whether U_LIST is valid
SET U_START_TIME TO 0.       // time origin for U selection
SET U_TF TO 0.               // total time span for U_LIST
SET U_T0 TO 0.               // first absolute t from solver
SET REMAIN_TF TO -1.         // remain tf from cpp; -1 means unknown
SET LAST_TICK TO TIME:SECONDS.
SET LAST_U_TICK TO 0.
SET LAST_PRINT_TICK TO 0.
SET RUN_ACTIVE TO 1.
SET GEAR_DEPLOYED TO 0.      // one-shot landing gear deploy latch

SET CONFIG:ipu TO 2000.

// Build a world-space vector from local ENU components using heading().

// On start, clear send.txt if it exists
IF EXISTS(SEND_FILE) { DELETEPATH(SEND_FILE). }.
// Also clear stale receive.txt to avoid running old/invalid commands
IF EXISTS(RECV_FILE) { DELETEPATH(RECV_FILE). }.

// Startup vehicle mode setup requested by flight procedure.
STAGE.
RCS ON.
SAS OFF.

// Build ENU basis vectors (up, north, east) from a position vector.
FUNCTION GET_LH_ENU_AXES {
  PARAMETER R0.

  LOCAL UP_SIDE IS R0:NORMALIZED.              // z'
  LOCAL NORTH_POLE IS BODY:NORTH:VECTOR.

  // 右手叉乘得到 EAST
  LOCAL EAST_SIDE IS VCRS(NORTH_POLE, UP_SIDE).

  // 极点退化保护（可选，但很推荐）
  IF EAST_SIDE:MAG < 1E-8 {
    // 如果 UP 几乎与 NORTH_POLE 平行，换一个参考轴避免 EAST 为零
    LOCAL REF IS V(1,0,0).
    LOCAL EAST_SIDE IS VCRS(REF, UP_SIDE).
  }
  SET EAST_SIDE TO EAST_SIDE:NORMALIZED.       // x'

  LOCAL NORTH_SIDE IS VCRS(UP_SIDE, EAST_SIDE):NORMALIZED. // 这是“右手北”

  // 左手系修正：把 y' 取为 -NORTH_SIDE
  LOCAL Y_SIDE IS NORTH_SIDE.               // y'
  LOCAL X_SIDE IS EAST_SIDE.                   // x'
  LOCAL Z_SIDE IS -UP_SIDE.                     // z'

  RETURN LIST(X_SIDE, Y_SIDE, Z_SIDE).
}

FUNCTION XYZ_TO_PFRAME {
  PARAMETER VXYZ, AXES.
  // AXES = LIST(X_SIDE, Y_SIDE, Z_SIDE)

  LOCAL X_SIDE IS AXES[0].
  LOCAL Y_SIDE IS AXES[1].
  LOCAL Z_SIDE IS AXES[2].

  LOCAL VX_P IS VDOT(VXYZ, X_SIDE).
  LOCAL VY_P IS VDOT(VXYZ, Y_SIDE).
  LOCAL VZ_P IS VDOT(VXYZ, Z_SIDE).

  RETURN V(VX_P, VY_P, VZ_P).
}
FUNCTION PFRAME_TO_XYZ {
  PARAMETER VP, AXES.
  // VP = V(vx', vy', vz')
  // AXES = LIST(X_SIDE, Y_SIDE, Z_SIDE)

  LOCAL X_SIDE IS AXES[0].
  LOCAL Y_SIDE IS AXES[1].
  LOCAL Z_SIDE IS AXES[2].

  LOCAL VXYZ IS (X_SIDE * VP:X) + (Y_SIDE * VP:Y) + (Z_SIDE * VP:Z).
  RETURN VXYZ.
}

// ===== Helper to compute ENU + velocity/state =====
// Build ENU state and vehicle parameters for solver input.
// ENU origin = target lat/lon + terrain height at target.
// Units: meters, m/s, N, s, kg.
FUNCTION CALC_STATE {
  SET rp TO BODY:GEOPOSITIONLATLNG(LAT0,LON0):POSITION.
  SET r0 TO BODY:position:normalized.

  LOCAL enu_axes IS GET_LH_ENU_AXES(r0).
  LOCAL up_side IS enu_axes[0].
  LOCAL north_side IS enu_axes[1].
  LOCAL east_side IS enu_axes[2].
  
  SET M TO XYZ_TO_PFRAME(-rp, enu_axes).
  SET EAST_M TO M:x.
  SET NORTH_M TO M:y.
  SET UP_M TO M:z.

  SET vel TO SHIP:VELOCITY:SURFACE.
  SET vel_P TO XYZ_TO_PFRAME(vel, enu_axes).

  LOCAL VU IS vel_P:z.
  LOCAL VN IS vel_P:y.
  LOCAL VE IS vel_P:x.
  LOCAL mag_vel IS SQRT(VE * VE + VN * VN + VU * VU).

  LOCAL ATM_AT IS SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE).
  LOCAL THRUST_MAX IS SHIP:AVAILABLETHRUSTAT(ATM_AT).
  LOCAL ISP_CUR IS 0.
  LIST ENGINES IN ENGS.
  IF ENGS:LENGTH > 0 {
    LOCAL ENG IS ENGS[0].
    FOR E IN ENGS {
      IF E:AVAILABLETHRUST > 0 { SET ENG TO E. BREAK. }.
    }.
    SET ISP_CUR TO ENG:ISPAT(ATM_AT).
  }.
  LOCAL MASS_WET IS SHIP:MASS.

  LOCAL DATA IS LIST().
  DATA:ADD(EAST_M).
  DATA:ADD(NORTH_M).
  DATA:ADD(UP_M).
  DATA:ADD(VE).
  DATA:ADD(VN).
  DATA:ADD(VU).
  DATA:ADD(mag_vel).
  DATA:ADD(THRUST_MAX).
  DATA:ADD(ISP_CUR).
  DATA:ADD(MASS_WET).
  RETURN DATA.
}

// Update thrust command from vector components (up, north, east).
// Magnitude is acceleration (m/s^2); throttle = |U| * mass / available_thrust.
// Yaw/Pitch are derived here; steering/throttle are locked outside this function.
FUNCTION APPLY_THRUST {
  PARAMETER U_UP, U_NORTH, U_EAST.
  SET r0 TO BODY:position:normalized.
  LOCAL enu_axes IS GET_LH_ENU_AXES(r0).
  
  SET STEER_CMD TO PFRAME_TO_XYZ(V(U_EAST, U_NORTH, U_UP), enu_axes).          // 反向推力向量在世界坐标系下
  LOCAL U_MAG IS SQRT(U_UP * U_UP + U_NORTH * U_NORTH + U_EAST * U_EAST).

  IF U_MAG <> U_MAG {
    SET LAST_THR_CMD TO 0.
    SET HAS_CUR_U TO 0.
    RETURN.
  }.
  LOCAL THRUST_AVAIL IS SHIP:AVAILABLETHRUSTAT(SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE)). // kN
  IF U_MAG <= 0 OR THRUST_AVAIL <= 0 {
    SET LAST_THR_CMD TO 0.
    SET HAS_CUR_U TO 0.
    RETURN.
  }.
  LOCAL THR_CMD IS U_MAG * SHIP:MASS / (THRUST_AVAIL).
  SET LAST_THR_CMD TO THR_CMD.
  SET HAS_CUR_U TO 1.
}

FUNCTION TRIGGER_U_ZERO {
  SET LAST_THR_CMD TO 0.
  SET HAS_CUR_U TO 0.
  SET U_MAG TO 0.
  SET RUN_ACTIVE TO 0.
  SET STEER_CMD TO PFRAME_TO_XYZ(V(0,0,1), GET_LH_ENU_AXES(BODY:POSITION:NORMALIZED)).
  LOCK STEERING TO STEER_CMD.
  WAIT 3.
  RCS OFF.
}

// Send current state to PC.
// INFO_IN: 0 for initial solve, 1 for mode1 recompute.
FUNCTION SEND_STATE {
  PARAMETER INFO_IN.
  IF RUN_ACTIVE = 0 { SET INFO_IN TO "END". }.
  SET STATE TO CALC_STATE().
  SET EAST_M TO STATE[0].
  SET NORTH_M TO STATE[1].
  SET UP_M TO STATE[2].
  SET VE TO STATE[3].
  SET VN TO STATE[4].
  SET VU TO STATE[5].
  SET SPEED TO STATE[6].
  SET THRUST_MAX TO STATE[7].
  SET ISP_CUR TO STATE[8].
  SET MASS_WET TO STATE[9].
  SET OUT_LINE TO INFO_IN + "," +
                ROUND(UP_M + UP_BIAS_M,1) + "," +
                ROUND(NORTH_M,1) + "," +
                ROUND(EAST_M,1) + "," +
                ROUND(VU,2) + "," +
                ROUND(VN,2) + "," +
                ROUND(VE,2) + "," +
                ROUND(SPEED,2) + "," +
                ROUND(THRUST_MAX,3) + "," +
                ROUND(ISP_CUR,3) + "," +
                ROUND(MASS_WET,3) + "," +
                ROUND(THROT1,2) + "," +
                ROUND(THROT2,2) + "," +
                ROUND(THETA_DEG,1) + "," +
                ROUND(YGS_DEG,1) + "," +
                ROUND(ELAPSED_SEC,2).

  LOG OUT_LINE TO SEND_FILE.
  SET SEND_STATE_TIME TO TIME:SECONDS.
//  PRINT "SEND MODE" + INFO_IN + ": " + OUT_LINE.
}

// Read solver output from receive.txt and refresh U_LIST if complete.
FUNCTION READ_SOLVER_OUTPUT {
  IF NOT EXISTS(RECV_FILE) { RETURN. }.
  SET RECV_LINE TO "".
  UNTIL RECV_LINE <> "" {
    SET RECV_LINE TO OPEN(RECV_FILE):READALL:STRING.
    IF RECV_LINE = "" { WAIT 0. }.
  }.

  SET COMPUTE_FINISH TO 0.
  SET HAS_FINISH TO 0.
  SET FINISH_CODE TO 0.
  SET U_COUNT TO 0.
  SET NEW_U_LIST TO LIST().
  SET HAS_NEW_REMAIN_TF TO 0.
  SET NEW_REMAIN_TF TO REMAIN_TF.

  SET RECV_LINE TO RECV_LINE:REPLACE(CR, "").
  SET RECV_LINES TO RECV_LINE:SPLIT(LF).
  SET t_loop_start_ms TO ROUND(TIME:SECONDS * 1000, 0).
  FOR L IN RECV_LINES {
    IF L <> "" {
      SET PARTS TO L:SPLIT(",").
      IF PARTS:LENGTH >= 2 {
        IF PARTS[0] = "COMPUTE_FINISH" AND PARTS[1]:STARTSWITH("1") {
          SET HAS_FINISH TO 1.
          SET FINISH_CODE TO 1.
        } ELSE IF PARTS[0] = "COMPUTE_FINISH" AND PARTS[1]:STARTSWITH("2") {
          SET HAS_FINISH TO 1.
          SET FINISH_CODE TO 2.
        } ELSE IF PARTS[0] = "U" AND PARTS:LENGTH >= 5 {
          SET U_UP TO PARTS[1]:TONUMBER.
          SET U_NORTH TO PARTS[2]:TONUMBER.
          SET U_EAST TO PARTS[3]:TONUMBER.
          SET U_T TO PARTS[4]:TONUMBER.
          NEW_U_LIST:ADD(LIST(U_UP, U_NORTH, U_EAST, U_T)).
          SET U_COUNT TO U_COUNT + 1.
        } ELSE IF PARTS[0] = "REMAIN_TF" AND PARTS:LENGTH >= 2 {
          SET NEW_REMAIN_TF TO PARTS[1]:TONUMBER.
          SET HAS_NEW_REMAIN_TF TO 1.
        }.
      }.
    }.
  }.
  SET t_loop_end_ms TO ROUND(TIME:SECONDS * 1000, 0).
  SET loop_ms TO t_loop_end_ms - t_loop_start_ms.
//  PRINT "recv_lines loop(ms)=" + loop_ms + " lines=" + RECV_LINES:LENGTH.

  IF HAS_FINISH = 1 {
    IF FINISH_CODE = 2 {
      SET RECOMPUTE_ENABLED TO 0.
      SET RECOMPUTE_PENDING TO 0.
      SET REMAIN_TF TO -1.
//      PRINT "RECOMPUTE_DISABLED_BY_PC".
      SET rf2 TO OPEN(RECV_FILE).
      rf2:CLEAR().
      DELETEPATH(RECV_FILE).
      RETURN.
    }.
    SET NEW_VALID TO 0.
    IF U_COUNT > 0 { SET NEW_VALID TO 1. }.
    IF NEW_VALID = 1 {
      SET U_LIST TO NEW_U_LIST.
      SET HAS_U_LIST TO 1.
      SET HAS_CUR_U TO 0.
      SET U_START_TIME TO ELAPSED_SEC.
      SET U_T0 TO U_LIST[0][3].
      // derive total horizon from first/last U time
      IF U_LIST:LENGTH > 1 {
        SET U_TF TO U_LIST[U_LIST:LENGTH - 1][3] - U_LIST[0][3].
        IF U_TF < 0 { SET U_TF TO -U_TF. }.
        IF U_TF = 0 { SET U_TF TO LOOP_DT. }.
      } ELSE {
        SET U_TF TO LOOP_DT.
      }.
      IF HAS_NEW_REMAIN_TF = 1 {
        SET REMAIN_TF TO NEW_REMAIN_TF.
      } ELSE {
        SET REMAIN_TF TO U_TF.
      }.
//      PRINT "U_LIST updated: len=" + U_LIST:LENGTH + " tf=" + U_TF.
//      PRINT "TF_10_LINES=" + U_TF.
    }.
    SET RECOMPUTE_PENDING TO 0.
    IF NEW_VALID = 1 {
      SET rf TO OPEN(RECV_FILE).
      rf:CLEAR().
      DELETEPATH(RECV_FILE).
    }.
  }.
}

// Select U based on elapsed time since U_START_TIME (no U_INDEX).
FUNCTION SELECT_U {
  IF HAS_U_LIST = 0 { RETURN. }.
  // derive horizon each select to be robust to changed lists
  SET U_TF TO 0.
  IF U_LIST:LENGTH > 1 {
    SET U_TF TO U_LIST[U_LIST:LENGTH - 1][3] - U_LIST[0][3].
    IF U_TF < 0 { SET U_TF TO -U_TF. }.
  } ELSE {
    SET U_TF TO LOOP_DT.
  }.
  IF U_TF <= 0 { RETURN. }.

  LOCAL t_elapsed IS ELAPSED_SEC - U_START_TIME.
  IF t_elapsed < 0 { SET t_elapsed TO 0. }.
  // Map kOS elapsed to solver absolute timeline starting at U_T0
  LOCAL t_abs IS U_T0 + t_elapsed.
  LOCAL idx IS FLOOR(((t_abs - U_T0) / U_TF) * U_LIST:LENGTH).
  IF idx < 0 { SET idx TO 0. }.
  IF idx >= U_LIST:LENGTH { SET idx TO U_LIST:LENGTH - 1. }.
  LOCAL U_CUR IS U_LIST[idx].
  LOCAL U_UP IS U_CUR[0].
  LOCAL U_NORTH IS U_CUR[1].
  LOCAL U_EAST IS U_CUR[2].
  SET U_MAG TO SQRT(U_UP * U_UP + U_NORTH * U_NORTH + U_EAST * U_EAST).
  IF U_MAG <= 0 {
//    PRINT "U_ZERO_STOP".
    TRIGGER_U_ZERO().
//    PRINT "PROGRAM_END".
    RETURN.
  }.
  APPLY_THRUST(U_UP, U_NORTH, U_EAST).
}

// ===== Single-shot mode0 compute and write =====
IF EXISTS(SEND_FILE) { DELETEPATH(SEND_FILE). }.
SEND_STATE("0").
//PRINT "SEND MODE0".

// ===== Wait for initial mode0 response =====
// Block until PC creates receive.txt (mode0 done). Then delete it and continue.
UNTIL EXISTS(RECV_FILE) {
  WAIT 0.
}.
SET RF0_LINE TO OPEN(RECV_FILE):READALL:STRING.
IF RF0_LINE:CONTAINS("COMPUTE_FINISH,2") {
  SET RECOMPUTE_ENABLED TO 0.
  SET RECOMPUTE_PENDING TO 0.
//  PRINT "RECOMPUTE_DISABLED_BY_PC_INIT".
}.
SET rf0 TO OPEN(RECV_FILE).
rf0:CLEAR().
DELETEPATH(RECV_FILE).
SET TIME0_SEC TO TIME:SECONDS.
SET LOOP_INDEX TO 0.
SET HAS_U_LIST TO 0.

// ===== Main tick (triggered by WHEN) =====
FUNCTION MAIN_TICK {
  IF RUN_ACTIVE = 0 { RETURN. }.
  SET NOW_SEC TO TIME:SECONDS.
  SET ELAPSED_SEC TO NOW_SEC - TIME0_SEC.

  // Read raw state once per tick; UP_M here is un-biased.
  SET STATE TO CALC_STATE().
  SET EAST_M TO STATE[0].
  SET NORTH_M TO STATE[1].
  SET UP_M TO STATE[2].
  SET SPEED TO STATE[6].

  IF SPEED < 5 {
    TRIGGER_U_ZERO().
    RETURN.
  }.

  IF UP_M < CUTDOWN_ALTITUDE {
    SET HAS_U_LIST TO 0.
    SET RECOMPUTE_ENABLED TO 0.
    SET RECOMPUTE_PENDING TO 0.
//    PRINT "CUTDOWN_TRIGGER up=" + ROUND(UP_M,2) + " cutoff=" + ROUND(CUTDOWN_ALTITUDE,2).
//    PRINT "PROGRAM_END".
    TRIGGER_U_ZERO().
    RETURN.
  }.

  IF GEAR_DEPLOYED = 0 AND REMAIN_TF >= 0 AND REMAIN_TF <= DEPLOY_GEAR_TIME {
    GEAR ON.
    SET GEAR_DEPLOYED TO 1.
//    PRINT "GEAR_DEPLOY remain_tf=" + ROUND(REMAIN_TF,2).
  }.

  // On first tick, send initial mode1 config.
  IF LOOP_INDEX = 0 AND RECOMPUTE_ENABLED = 1 {
    SEND_STATE("1").
    SET RECOMPUTE_PENDING TO 1.
  }.

  // Periodic recompute trigger based on elapsed time.
  IF RECOMPUTE_ENABLED = 1 AND RECOMPUTE_PENDING = 0 AND (ELAPSED_SEC - LAST_RECOMPUTE_TIME) >= RECOMPUTE_TIME {
//    PRINT "RECOMPUTE_TRIGGER t=" + ROUND(ELAPSED_SEC,2).
    SEND_STATE("1").
    SET RECOMPUTE_PENDING TO 1.
    SET LAST_RECOMPUTE_TIME TO ELAPSED_SEC.
  }.

  SET LOOP_INDEX TO LOOP_INDEX + 1.

  // Telemetry values are printed by dedicated PRINT_TICK() WHEN.
  SET ERR_M TO SQRT(EAST_M * EAST_M + NORTH_M * NORTH_M).
//  PRINT "t=" + ROUND(ELAPSED_SEC,2) +
//        " err=" + ROUND(ERR_M,1) +
//        " up=" + ROUND(UP_M,2) +
//        " mass=" + ROUND(SHIP:MASS,3).

  READ_SOLVER_OUTPUT().
}

// ===== Thruster/steering tick (same cadence as MAIN_TICK) =====
FUNCTION U_TICK {
  IF RUN_ACTIVE = 0 { RETURN. }.
  SET NOW_SEC TO TIME:SECONDS.
  SET ELAPSED_SEC TO NOW_SEC - TIME0_SEC.
  IF HAS_U_LIST = 1 {
    SELECT_U().
  }.
}

// ===== Dedicated print tick =====
FUNCTION PRINT_TICK {
  CLEARSCREEN.
  PRINT "t=" + ROUND(ELAPSED_SEC,2).
  PRINT "loop_time=" + ROUND(LOOP_TIME,4).
  PRINT "err=" + ROUND(ERR_M,1).
  PRINT "up=" + ROUND(UP_M,2).
  PRINT "u=" + ROUND(U_MAG,2).
  PRINT "speed=" + ROUND(SPEED,2).
  PRINT "remain_tf=" + ROUND(REMAIN_TF,3).
  PRINT "mass=" + ROUND(SHIP:MASS,3).
}

// ===== Schedule tick via WHEN =====
SET LAST_TICK TO TIME:SECONDS.
SET LAST_U_TICK TO TIME:SECONDS.
SET LAST_PRINT_TICK TO TIME:SECONDS.
LOCK THROTTLE TO LAST_THR_CMD.
LOCK STEERING TO LOOKDIRUP(STEER_CMD, -BODY:NORTH:VECTOR).
WHEN RUN_ACTIVE = 1 AND TIME:SECONDS >= LAST_TICK + LOOP_DT THEN {
  SET LAST_TICK TO TIME:SECONDS.
  MAIN_TICK().
  PRESERVE.
}.
WHEN RUN_ACTIVE = 1 AND TIME:SECONDS >= LAST_U_TICK + LOOP_DT THEN {
  SET LOOP_TIME TO TIME:SECONDS-LAST_U_TICK.
  SET LAST_U_TICK TO TIME:SECONDS.
  U_TICK().
  PRESERVE.
}.
WHEN RUN_ACTIVE = 1 AND TIME:SECONDS >= SEND_STATE_TIME + RESEND_TIME THEN {
  SEND_STATE("INFO").
  PRESERVE.
}.
WHEN RUN_ACTIVE = 1 AND TIME:SECONDS >= LAST_PRINT_TICK + LOOP_DT THEN {
  SET LAST_PRINT_TICK TO TIME:SECONDS.
  PRINT_TICK().
  PRESERVE.
}.
WHEN RUN_ACTIVE = 0 THEN {
  SEND_STATE("END").
}.
WAIT UNTIL FALSE.
