package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.math.util.Units;

public class CEEConstants {
  // REAL CONSTANTS
  /** CAN ID of the CEE SPARK MAX */
  public static final int CAN_ID = 19;
  /** Gear reduction of 3:1 for the CEE motor */
  public static final double GEAR_RATIO = 3.0 / 1.0;
  /**
   * Set the inversion status of the CEE to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit, in amps for the CEE motor */
  public static final int CUR_LIM_A = 20;
  // Velocities
  /** Scoring velocity in radians per second */
  public static final double SCORE_VELOCITY_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(-30); // TODO: Update
  /** Intaking velocity in radians per second */
  public static final double INTAKE_VELOCITY_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(30); // TODO: Update
  /** Scoring speed, open loop voltage control */
  public static final double SCORE_PERCENT_SPEED = -0.80; // TODO: Verify
  /** Intaking speed, open loop voltage control */
  public static final double INTAKE_PERCENT_SPEED = 0.80; // TODO: Verify

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
