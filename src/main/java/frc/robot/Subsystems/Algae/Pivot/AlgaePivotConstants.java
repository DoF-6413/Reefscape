package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.util.Units;

public class AlgaePivotConstants {
  // REAL CONSTANTS
  /** CAN ID for the ALGAE Pivot SPARK MAX */
  public static final int CAN_ID = 21;
  /** Current limit, in amps, for the NEO motor of the ALGAE Pivot */
  public static final int CUR_LIM_A = 30;
  /** Enable current limiting for the ALGAE Pivot motor */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Set the inversion of the ALGAE Pivot motor to false, making CCW positive */
  public static final boolean IS_INVERTED = false;
  /** Gear reduction of 140:11 for the ALGAE Pivot motor */
  public static final double GEAR_RATIO = 140.0 / 11.0;
  /** Starting angle of the Pivot motor in radians */
  public static final double STARTING_ANGLE_RAD = 0.0;
  /** Maximum angle of the ALGAE Pivot in radians */
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(45);
  /** Minimum angle of the ALGAE Pivot in radians */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-35);
  /** Length of the ALGAE Pivot in meters */
  public static final double LENGTH_M = Units.inchesToMeters(1); // TODO: update with actual length

  // PID CONSTANTS
  // TODO: test PID values
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
  /** Moment of Inertia for the ALGAE Pivot in kilograms * meters squared */
  public static final double MOI_KG_M2 = 0.0001; // TODO: get actual MOI
  /** Simulate the pull of gravity in the ALGAE Pivot simulation */
  public static final boolean SIMULATE_GRAVITY = true;
}
