package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaePivotConstants {
  // REAL CONSTANTS
  /** CAN ID for the ALGAE Pivot SPARK MAX */
  public static final int CAN_ID = 19;
  /** Gear reduction of 140:11 for the ALGAE Pivot motor */
  public static final double GEAR_RATIO = 140.0 / 11.0;
  /**
   * Set the inversion of the ALGAE Pivot motor to false, making Counterclockwise the positive
   * direction
   */
  public static final boolean IS_INVERTED = false;
  /** Current limit for the NEO motor of the ALGAE Pivot */
  public static final int CUR_LIM_A = 30;
  /** Offset to zero Absolute Encoder to be parallel with the Drivetraion belly pan, in rotations */
  public static final double ZERO_OFFSET = 0.0;
  /** Length of the ALGAE Pivot in meters */
  public static final double LENGTH_M = Units.inchesToMeters(10.151);
  /** Weight of the ALGAE Pivot in kilograms */
  public static final double MASS_KG = Units.lbsToKilograms(6.8);
  // Angle positions
  /** Starting angle of the ALGAE Pivot in radians */
  public static final double DEFAULT_ANGLE_RAD = 0.0;
  /** Minimum angle of the ALGAE Pivot in radians */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-35);
  /** Maximum angle of the ALGAE Pivot in radians */
  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(75);
  /** Angle (radians) of the ALGAE Pivot when trying to pickup ALGAE off the REEF */
  public static final double REEF_ALGAE_ANGLE_RAD = Units.degreesToRadians(12.5);
  /** Angle (radians) of the ALGAE Pivot when trying to pickup ALGAE off the ground */
  public static final double GROUND_ALGAE_ANGLE_RAD = Units.degreesToRadians(0); // TODO: Update
  /** Angle (radians) of the ALGAE Pivot when trying to score ALGAE at the NET */
  public static final double NET_ANGLE_RAD = Units.degreesToRadians(45);
  /** Angle (radians) of the ALGAE Pivot when trying to score ALGAE at PROCESSOR */
  public static final double PROCESSOR_ANGLE_RAD = Units.degreesToRadians(0); // TODO: Update

  // PID CONSTANTS
  // TODO: Tune PID gains
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.1;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 0.225;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double KD = 0.05;
  /**
   * How many radians the angle of the ALGAE Pivot can be within its angle setpoint to be considered
   * at the setpoint
   */
  public static final double ERROR_TOLERANCE_RAD = Units.degreesToRadians(1); // TODO: test

  // SIM CONSTANTS
  /** Moment of inertia for the ALGAE Pivot in kilograms * meters squared */
  public static final double MOI_KG_M2 = SingleJointedArmSim.estimateMOI(LENGTH_M, MASS_KG);
  /** Simulate the pull of gravity in the ALGAE Pivot simulation */
  public static final boolean SIMULATE_GRAVITY = false;
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double KP_SIM = 1.1;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static final double KI_SIM = 0.225;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static final double KD_SIM = 0.05;
}
