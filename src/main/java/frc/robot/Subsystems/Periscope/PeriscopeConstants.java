package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.util.Units;

public class PeriscopeConstants {
  // REAL CONSTANTS
  /** CAN ID for the first Periscope motor. This motor will use index 0 in any array */
  public static final int CAN_ID_LEFT = 15;
  /** CAN ID for the second Periscope motor. This motor will use index 1 in any array */
  public static final int CAN_ID_RIGHT = 16;
  /**
   * DIO ports of the 6 Hall Effect sensors on the Periscope.
   *
   * <p>0 - Bottom, 1 - Top
   */
  public static final int[] HALL_EFFECT_SENSORS_PORTS = {0, 1};
  /** Gear reduction of 38:12 for the Periscope */
  public static final double GEAR_RATIO = 38.0 / 12.0;
  /**
   * Sets the inversion status of the lead Periscope motor to false, making Counterclockwise the
   * positive direction
   */
  public static final boolean IS_INVERTED = true;
  /** Sets the follower Periscope motor to spin opposite of the lead motor */
  public static final boolean IS_INVERTED_FOLLOWER = true;
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
  /** Height position of the Periscope when is reaching L1 */
  public static final double L1_HEIGHT_M = Units.inchesToMeters(8.18);
  /** Height position of the Periscope when is reaching L2 */
  public static final double L2_HEIGHT_M = Units.inchesToMeters(19.57);
  /** Height position of the Periscope when is reaching L3 */
  public static final double L3_HEIGHT_M = Units.inchesToMeters(33.33);
  /** Height position of the Periscope when is reaching L4 */
  public static final double L4_HEIGHT_M = MAX_HEIGHT_M;
  /** Height position of the Periscope when is reaching the CORAL Station */
  public static final double CORAL_STATION_HEIGHT_M = Units.inchesToMeters(0.0); // TODO: Update
  /** Height position of the Periscope when is reaching the PROCCESOR */
  public static final double PROCESSOR_HEIGHT_M = Units.inchesToMeters(0.0); // TODO: Update
  /** Height position of the Periscope when is reaching the NET */
  public static final double NET_HEIGHT_M = MAX_HEIGHT_M;

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
  public static double KG = 0.18;
  /** KV represents the voltage used every second per meter */
  public static double KV = 5.08;
  /** KA represents the voltage used every second squared per meter */
  public static double KA = 0.03;
  /**
   * Max velocity for trapezoidal motion profiling in rotations per second. 2.32 = max velocity, in
   * meters per second, calculated from https://www.reca.lc/ assuming 85% efficiency
   */
  public static final double MAX_VELOCITY_ROT_PER_SEC =
      Units.radiansToRotations(2.32 / DRUM_RADIUS_M);
  /** Ideal acceleration for trapezoidal motion profiling in rotations per second squared */
  public static final double IDEAL_ACCELERATION_ROT_PER_SEC2 =
      Units.radiansToRotations(4 / DRUM_RADIUS_M);
  /**
   * How many meters the height of the Periscope can be within its height setpoint to be considered
   * at the setpoint
   */
  public static final double ERROR_TOLERANCE_M = Units.inchesToMeters(2); // TODO: test

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
  public static final double KG_SIM = 0.18;
  /** KV represents the voltage used every second per meter */
  public static final double KV_SIM = 5.08;
  /** KA represents the voltage used every second squared per meter */
  public static final double KA_SIM = 0.03;
}
