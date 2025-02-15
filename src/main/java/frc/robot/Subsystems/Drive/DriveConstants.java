package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // REAL CONSTANTS
  /**
   * Proportion of error from Drive relative encoder readings to actual distance travelled by the
   * wheel due to the bolts holding down the tread
   */
  public static final double WHEEL_RADIUS_ERROR_COEFF = 1 - 0.050793;
  /** Radius of the wheel in meters */
  public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2) * WHEEL_RADIUS_ERROR_COEFF;
  /** Side length of the robot in meters */
  public static final double TRACK_WIDTH_M = Units.inchesToMeters(29);
  /** Radius of the robot (diagonal) in meters */
  public static final double DRIVETRAIN_RADIUS_M = Math.hypot(TRACK_WIDTH_M / 2, TRACK_WIDTH_M / 2);
  /** Gear Ratio for MK4i L3 Krakens */
  public static final double DRIVE_GEAR_RATIO = 6.12;
  /** Gear Ratio for MK4i Neos */
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double DRIVE_KP = 0.3;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double DRIVE_KI = 0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double DRIVE_KD = 0.001;
  /** KS represents the voltage required to overcome static friction */
  public static double DRIVE_KS = 0.123;
  /** KV represents the voltage used every second per meter */
  public static double DRIVE_KV = 0.1;
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double TURN_KP = 6.4;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double TURN_KI = 0.0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static double TURN_KD = 0.05;

  /** Max Linear Speed of Robot */
  public static final double MAX_LINEAR_SPEED_M_PER_S = 5.2; // TODO: Update? Since robot is larger
  /** Set the inverted for the turn SparkMax */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_S =
      MAX_LINEAR_SPEED_M_PER_S / DRIVETRAIN_RADIUS_M;

  public static final boolean TURN_IS_INVERTED = true;
  /** the update frequency */
  public static final double UPDATE_FREQUENCY_HZ = 100;
  /** Current limiting in amps */
  public static final int CUR_LIM_A = 60;
  /** Enables the current limit */
  public static final boolean ENABLE_CUR_LIM = true;
  /**
   * Within 10% of the desired direction, the joystick is considered to be going in that direction
   */
  public static final double DEADBAND = 0.1;

  // SIM CONSTANTS
  /** Moment of Inertia value for Drive motor - Module wheel system in kilograms * meters squared */
  public static final double DRIVE_MOI_KG_M2 = 0.0003125;
  /** Moment of Inertia value for Turn motor - Module wheel system in kilograms * meters squared */
  public static final double TURN_MOI_KG_M2 = 0.0000158025413;
  /** KP value for the simulated Drive motor as the simulation conditions are ideal and static */
  public static double DRIVE_KP_SIM = 0.01;
  /** KI value for the simulated Drive motor as the simulation conditions are ideal and static */
  public static double DRIVE_KI_SIM = 0.0;
  /** KD value for the simulated Drive motor as the simulation conditions are ideal and static */
  public static double DRIVE_KD_SIM = 0.0001;
  /** KS value for the simulated Drive motor as the simulation conditions are ideal and static */
  public static double DRIVE_KS_SIM = 0.123;
  /** KV value for the simulated Drive motor as the simulation conditions are ideal and static */
  public static double DRIVE_KV_SIM = 0.1;

  /**
   * Translation 2d assumes that the robot front facing is in the positive x direction and the robot
   * left is in the positive y direction
   *
   * <p>Directions of the VECTORS that each module turns in (i.e. Module 0, front right, when the
   * robot spins ccw, will point towards Quadrant II which is (-,+))
   */
  public static final Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
    };
  }

  // KrakenX60 CAN IDs
  public enum DRIVE_MOTOR {
    FRONT_RIGHT(2), // Module 0
    FRONT_LEFT(3), // Module 1
    BACK_LEFT(4), // Module 2
    BACK_RIGHT(5); // Module 3

    public final int CAN_ID;

    DRIVE_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  // NEOs CAN IDs
  public enum TURN_MOTOR {
    FRONT_RIGHT(6), // Module 0
    FRONT_LEFT(7), // Module 1
    BACK_LEFT(8), // Module 2
    BACK_RIGHT(9); // Module 3

    public final int CAN_ID;

    TURN_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  // CANcoders CAN IDs
  public enum ABSOLUTE_ENCODER {
    FRONT_RIGHT(10), // Module 0
    FRONT_LEFT(11), // Module 1
    BACK_LEFT(12), // Module 2
    BACK_RIGHT(13); // Module 3

    public final int CAN_ID;

    ABSOLUTE_ENCODER(int value) {
      CAN_ID = value;
    }
  }

  public enum ABSOLUTE_ENCODER_OFFSET {
    FRONT_RIGHT(1.88059248437), // Module 0
    FRONT_LEFT(1.16695161013), // Module 1
    BACK_LEFT(-0.60147581563), // Module 2
    BACK_RIGHT(1.04951470522); // Module 3

    public final double OFFSET;

    ABSOLUTE_ENCODER_OFFSET(double value) {
      OFFSET = value;
    }
  }
}
