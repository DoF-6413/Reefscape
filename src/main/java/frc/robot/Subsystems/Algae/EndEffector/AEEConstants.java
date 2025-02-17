package frc.robot.Subsystems.Algae.EndEffector;

public class AEEConstants {
  // REAL CONSTANTS
  /** CAN ID for the AEE motor */
  public static final int CAN_ID = 20;
  /** Current limit for the NEO motor of the AEE */
  public static final int CUR_LIM_A = 20;
  /** Set the inversion status of the AEE to false, making CCW positive */
  public static final boolean IS_INVERTED = false;
  /** Gear ratio of 6:1 for the AEE motor */
  public static final double GEAR_RATIO = 6.0 / 1.0;

  // PID CONSTANTS
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

  // SIM CONSTANTS
  /** Moment of Inertia for the AEE wheels in kilograms * meters squared */
  public static final double MOI_KG_M2 = 0.0025;
}
