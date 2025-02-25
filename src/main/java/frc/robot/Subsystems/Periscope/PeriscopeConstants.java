package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.util.Units;

public class PeriscopeConstants {
  // REAL CONSTANTS
  /** CAN ID for the first Periscope motor. This motor will use index 0 in any array */
  public static final int CAN_ID_0 = 15;
  /** CAN ID for the second Periscope motor. This motor will use index 1 in any array */
  public static final int CAN_ID_1 = 16;
  /** Port where the hall effect sensors */
  public static final int[] HALL_EFFECT_SENSORS = {1,2,3,4,5,6};
  /** Gear reduction of 38:12 for the Periscope */
  public static final double GEAR_RATIO = 38.0 / 12.0;
  /**
   * Sets the inversion status of the Periscope motors to false, making Counterclockwise the
   * positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit of 60 amps for the Periscope motors */
  public static final int CUR_LIM_A = 60;
  /** Enable current limiting for the Periscope motors */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Refresh signals of the TalonFX 50 times a second (every 0.02 second) */
  public static final int UPDATE_FREQUENCY_HZ = 50;
  /** Radius of the driving drum of the Periscope, accounts for grooves for rope, in meters */
  public static final double DRUM_RADIUS_M = Units.inchesToMeters(0.9375);
  /** Mass of the carriage in kilograms */
  public static final double MASS_KG = Units.lbsToKilograms(20.0);
  // Height positions
  /** Minimum height of the Periscope will be 0 meters */
  public static final double MIN_HEIGHT_M = 0.0;
  /** Max height of the Periscope in meters */
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(57.925);

  // PROFILED PID & FEEDFORWARD CONSTANTS
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 0.0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double KD = 0.0;
  /** KG represents the voltage required to overcome static friction */
  public static double KS = 0.0;
  /** KG represents the voltage required to overcome gravity */
  public static double KG = 0.0;
  /** KV represents the voltage used every second per meter */
  public static double KV = 0.0;
  /** KA represents the voltage used every second squared per meter */
  public static double KA = 0.0;
  /**
   * Max velocity for trapezoidal motion profiling in rotations per second. 2.25 = max velocity, in
   * meters per second, calculated from https://www.reca.lc/ assuming 100% efficiency
   */
  public static final double MAX_VELOCITY_ROT_PER_SEC =
      Units.radiansToRotations(2.25 / DRUM_RADIUS_M);
  /** Ideal acceleration for trapezoidal motion profiling in rotations per second squared */
  public static final double IDEAL_ACCELERATION_ROT_PER_SEC2 = 0.0;

  // SIM CONSTANTS
  /** Simulate the pull of gravity in the elevator simulation */
  public static final boolean SIMULATE_GRAVITY = true;
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double KP_SIM = 30.0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static final double KI_SIM = 1.5;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static final double KD_SIM = 5.0;
  /** KG represents the voltage required to overcome static friction */
  public static final double KS_SIM = 0.0;
  /** KG represents the voltage required to overcome gravity */
  public static final double KG_SIM = 0.21;
  /** KV represents the voltage used every second per meter */
  public static final double KV_SIM = 5.25;
  /** KA represents the voltage used every second squared per meter */
  public static final double KA_SIM = 0.02;
}
