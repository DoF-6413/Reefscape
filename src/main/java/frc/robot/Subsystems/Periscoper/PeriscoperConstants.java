package frc.robot.Subsystems.Periscoper;

import edu.wpi.first.math.util.Units;

public class PeriscoperConstants {
  __
  public static final double GEAR_RATIO = 12.0 / 42.0;
  public static final double SPOOL_RADIUS_M = Units.inchesToMeters(0.625);
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(58); // meters
  public static final double MIN_HEIGHT_M = 0.0;
  public static final double MOI = 0.0; // TODO: Find MOI
  public static final boolean ENABLE_CUR_LIM = true;
  public static final int CUR_LIM_A = 60;
  public static final double MAX_VELOCITY_MPS = 1.4; // meters per second
  public static final double MASS_KG = Units.lbsToKilograms(30.0); // Kilograms
  public static final double MAX_ACCELERATION_MPS2 =
      0.0; // meters per second squared TODO: Find max acceleration
  public static final int CANID_1 = 15;
  public static final int CANID_2 = 16;
  public static final int UPDATE_FREQUENCY_HZ = 50;
  public static final double DRUM_RADIUS_M = Units.inchesToMeters(1.0);


  // PID & FEEDFORWARD CONSTANTS
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
  /** KS represents the voltage required to overcome static friction */
  public static double KS = 0.0;
  /** KV represents the voltage used every second per meter */
  public static double KV = 0.0;
  /** KV represents the voltage required to overcome gravity */
  public static double KG = 0.0;
}
