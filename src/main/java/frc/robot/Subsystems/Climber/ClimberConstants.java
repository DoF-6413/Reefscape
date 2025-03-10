package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class ClimberConstants {
  /** CAN ID for the first Climber motor */
  public static final int CAN_ID = 18;
  /** Gear reduction of 9:1 for the Climber */
  public static final double GEAR_RATIO = 9.0 / 1.0;
  /**
   * Set the inversion of the Climber motor to false, making Counterclockwise the positive direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit of 60 amps for the Climber motor */
  public static final int STALL_CUR_LIM_A = 100;
  /** Current limit of 120 amps for the Climber motor */
  public static final int MAX_CURRENT_A = 120;
  /** Enable current limiting for the Climber motor */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Refresh signals of the TalonFX 50 times a second (every 0.02 second) */
  public static final double UPDATE_FREQUENCY_HZ = 50;
  /** Length of the Climber arm in meters */
  public static final double LENGTH_M = Units.inchesToMeters(11.919);
  /** Weight of the Climber in kilograms */
  public static final double MASS_KG = Units.lbsToKilograms(10);
  // Climber Speeds
  /** Percent speed for deploying the Climber to initiate climbing */
  public static final double DEPLOY_PERCENT_SPEED = 0.2;
  /** Percent speed for retracting the Climber to pull up the robot */
  public static final double RETRACT_PERCENT_SPEED = -0.2;
  // Angle positions
  /** Minimum (default) angle of the Climber in radians */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(10);
  /** Maximum angle of the Climber in radians */
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(90);

  // PID Constants
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 2.0;
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
