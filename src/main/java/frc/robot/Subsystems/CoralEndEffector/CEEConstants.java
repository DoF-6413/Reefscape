package frc.robot.Subsystems.CoralEndEffector;

public class CEEConstants {
  // REAL CONSTANTS
  /** CAN ID of the CEE SPARK MAX */
  public static final int CAN_ID = 19;
  /** Current limit, in amps for the CEE motor */
  public static final int CUR_LIM_A = 20;
  /**
   * Set the inversion status of the CEE to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Gear reduction of 3:1 for the CEE motor */
  public static final double GEAR_RATIO = 3.0 / 1.0;
  /** Port where the beam breake is */
  public static final int BEAM_BREAK_PORT = 0;

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

  // SIM Constants
  /** Moment of inertia of the CEE wheels in kilograms * meters squared */
  public static final double MOI_KG_M2 = 0.0025;
}
