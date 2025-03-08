package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.util.Units;

public class FunnelConstants {
  // REAL CONSTANTS
  /** CAN ID of the Funnel SPARK MAX */
  public static final int CAN_ID = 17;
  /** Gear reduction of 3:1 for the Funnel motor */
  public static final double GEAR_RATIO = 3.0 / 1.0;
  /**
   * Set the inversion status of the Funnel to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = true;
  /** Current limit for the NEO motor of the Funnel */
  public static final int CUR_LIM_A = 30;
  // Velocities
  /** Intaking velocity in radians per second */
  public static final double INTAKE_SPEED_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(1000); // TODO: Update
  /** Intaking speed, open loop voltage control */
  public static final double INTAKE_PERCENT_SPEED = 0.65; // TODO: Verify

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
