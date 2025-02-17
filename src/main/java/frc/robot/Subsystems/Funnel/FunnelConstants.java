package frc.robot.Subsystems.Funnel;

public class FunnelConstants {
  // REAL CONSTANTS
  /** CAN ID of the Funnel Spark Max */
  public static final int CAN_ID = 18;
  /** Current limit, in amps for the Funnel Spark Max */
  public static final int CUR_LIM_A = 30;
  /** Set the inversion status of the Funnel to false, making CCW positive */
  public static final boolean IS_INVERTED = false;
  /** Gear reduction of 3:1 for the Funnel motor */
  public static final double GEAR_RATIO = 3.0 / 1.0;

  // PID CONSTANTS
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 0.0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double KD = 0.0;

  // SIM CONSTANTS
  /** Moment of Inertia of the Funnel wheels in kilograms * meters squared */
  public static final double MOI_KG_M2 = 0.0025;
}
