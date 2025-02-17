package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.util.Units;

public class PeriscopeConstants {
  // REAL CONSTANTS
  /** CAN ID for the first Periscope motor. This motor will use index 0 in any array */
  public static final int CAN_ID_0 = 15;
  /** CAN ID for the first Periscope motor. This motor will use index 1 in any array */
  public static final int CAN_ID_1 = 16;
  /** Gear reduction of 38:12 for the Periscope */
  public static final double GEAR_RATIO = 38.0 / 12.0;
  /** Current limit of 60 amps for the Periscope motors */
  public static final int CUR_LIM_A = 60;
  /** Enable current limiting for the Periscope motors */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Update the signals from the Periscope motors every 0.02 seconds */
  public static final int UPDATE_FREQUENCY_HZ = 50;
  /** Minimum height of the Periscope will be 0 meters */
  public static final double MIN_HEIGHT_M = 0.0;
  /** Max height of the Periscope in meters */
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(57.925);
  /** Radius of the driving drum of the Periscope, accounts for grooves for rope, in meters */
  public static final double DRUM_RADIUS_M = Units.inchesToMeters(0.9375);
  /** Mass of the carriage in kilograms */
  public static final double MASS_KG = Units.lbsToKilograms(20.0);

  // PROFILED PID & FEEDFORWARD CONSTANTS
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 30.0; // FROM SIM (!)(!)(!)
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 1.5; // FROM SIM (!)(!)(!)
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double KD = 5.0; // FROM SIM (!)(!)(!)
  /** KG represents the voltage required to overcome static friction */
  public static double KS = 0.0;
  /** KG represents the voltage required to overcome gravity */
  public static double KG = 0.21; // FROM RECALC (!)(!)(!)
  /** KV represents the voltage used every second per meter */
  public static double KV = 5.25; // FROM RECALC (!)(!)(!)
  /** KA represents the voltage used every second squared per meter */
  public static double KA = 0.02; // FROM RECALC (!)(!)(!)
  /**
   * Max velocity for trapezoidal motion profiling in rotations per second. 2.31 = max velocity, in
   * meters per second, calculated from https://www.reca.lc/ assuming 100% efficiency
   */
  public static final double MAX_VELOCITY_ROT_PER_SEC =
      Units.radiansToRotations(2.25 / DRUM_RADIUS_M);
  /** Ideal acceleration for trapezoidal motion profiling in rotations per second squared */
  public static final double IDEAL_ACCELERATION_ROT_PER_SEC_2 = 0.0;

  // SIM CONSTANTS
  /** Simulate the pull of gravity in the arm simulation */ 
  public static final boolean SIMULATE_GRAVITY = true;
}
