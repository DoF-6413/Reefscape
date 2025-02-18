package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ClimberConstants {
  /** CAN ID for the first Climber motor */
  public static final int CAN_ID = 17;
  /** Gear reduction of 9:1 for the Climber */
  public static final double GEAR_RATIO = 9.0 / 1.0;
  /**
   * Set the inversion of the Climber motor to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit of 50 amps for the Climber motor */
  public static final int CUR_LIM_A = 50;
  /** Enable current limiting for the Climber motor */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Update the signals from the Climber motor every 0.02 seconds */
  public static final double UPDATE_FREQUENCY_HZ = 50;
  /** Length of the Climber arm in meters */
  public static final double LENGTH_M = Units.inchesToMeters(11.919);
  /** Weight of the Climber in kilograms */
  public static final double MASS_KG = Units.lbsToKilograms(1);
  // Angle positions
  /** Minimum angle of the Climber in radians */
  public static final double CLIMBER_MIN_ANGLE_RAD = Units.degreesToRadians(10);
  /** Maximum (default) angle of the Climber in radians */
  public static final double CLIMBER_MAX_ANGLE_RAD = Units.degreesToRadians(90);

  // PID Constants
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
  /** Moment of inertia of the arm in kilograms * meters squared */
  public static final double MOI_KG_M2 = SingleJointedArmSim.estimateMOI(LENGTH_M, MASS_KG);
  /** Simulate the pull of gravity in the Climber arm simulation */
  public static final boolean SIMULATE_GRAVITY = true;
}
