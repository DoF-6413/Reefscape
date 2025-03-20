package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // REAL CONSTANTS
  /** KrakenX60 CAN IDs */
  public enum DRIVE_MOTOR {
    FRONT_LEFT(2), // Module 0
    FRONT_RIGHT(3), // Module 1
    BACK_LEFT(4), // Module 2
    BACK_RIGHT(5); // Module 3

    public final int CAN_ID;

    DRIVE_MOTOR(int value) {
      CAN_ID = value;
    }
  }
  /** CANcoders CAN IDs */
  public enum ABSOLUTE_ENCODER {
    FRONT_LEFT(6), // Module 0
    FRONT_RIGHT(7), // Module 1
    BACK_LEFT(8), // Module 2
    BACK_RIGHT(9); // Module 3

    public final int CAN_ID;

    ABSOLUTE_ENCODER(int value) {
      CAN_ID = value;
    }
  }
  /** NEOs CAN IDs */
  public enum TURN_MOTOR {
    FRONT_LEFT(10), // Module 0
    FRONT_RIGHT(11), // Module 1
    BACK_LEFT(12), // Module 2
    BACK_RIGHT(13); // Module 3

    public final int CAN_ID;

    TURN_MOTOR(int value) {
      CAN_ID = value;
    }
  }
  /** CAN ID for the Pigeon 2.0 IMU */
  public static final int GYRO_CAN_ID = 14;
  /** Gear Ratio for MK4i L3 Krakens */
  public static final double DRIVE_GEAR_RATIO = 6.12;
  /** Gear Ratio for MK4i Turn motors */
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
  /** Inversion status for the Drive motor, makes Counterclockwise the positive direction */
  public static final boolean DRIVE_IS_INVERTED = false;
  /** Inversion status for the Turn motor, makes Clockwise the positive direction */
  public static final boolean TURN_IS_INVERTED = true;
  /** Current limiting in amps */
  public static final int CUR_LIM_A = 40;
  /** Enables the current limit */
  public static final boolean ENABLE_CUR_LIM = true;
  /** Refresh signals of the TalonFX and CANcoder 50 times a second (every 0.02 second) */
  public static final double UPDATE_FREQUENCY_HZ = 50;
  /**
   * Refresh the position signals of the TalonFX and CANcoder 250 times a second (every 0.004
   * second)
   */
  public static final double ODOMETRY_UPDATE_FREQUENCY_HZ = 250;
  /**
   * Proportion of error from Drive relative encoder readings to actual distance travelled by the
   * wheel due to the bolts holding down the tread
   */
  public static final double WHEEL_RADIUS_ERROR_COEFF = 1 - 0.034925;
  /** Radius of the wheel in meters */
  public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2) * WHEEL_RADIUS_ERROR_COEFF;
  /** Side length of the robot in meters */
  public static final double TRACK_WIDTH_M = Units.inchesToMeters(29);
  /** Radius of the robot (diagonal) in meters */
  public static final double DRIVETRAIN_RADIUS_M = Math.hypot(TRACK_WIDTH_M / 2, TRACK_WIDTH_M / 2);
  /** Max linear speed of robot */
  public static final double MAX_LINEAR_SPEED_M_PER_S = 5.2;
  /** Max angular speed of the robot */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_S =
      MAX_LINEAR_SPEED_M_PER_S / DRIVETRAIN_RADIUS_M;
  /** Ingnore joystick inputs less than 10% tilted */
  public static final double DEADBAND = 0.1;
  /**
   * An angle to offset the heading of the Gyro to be the desired Front side of the robot. The Front
   * side will be defined as the scoring side for this robot
   */
  public static final double HEADING_OFFSET_RAD = Math.PI;
  /**
   * Offset the absolute position of the CANcoders to orientate the wheels to the front of the robot
   * at 0 degrees
   */
  public enum ABSOLUTE_ENCODER_OFFSET {
    FRONT_LEFT(0.4111068511533518 + (Math.PI / 2)), // Module 0
    FRONT_RIGHT(-2.6323110320117604 + (Math.PI / 2)), // Module 1
    BACK_LEFT(-1.4312040750973032 + (Math.PI / 2)), // Module 2
    BACK_RIGHT(-0.8820389530342436 + (Math.PI / 2)); // Module 3

    public final double OFFSET;

    ABSOLUTE_ENCODER_OFFSET(double value) {
      OFFSET = value;
    }
  }

  /**
   * Translation 2d assumes that the robot front facing is in the positive x direction and the robot
   * left is in the positive y direction.
   *
   * @return An array of the {@link Translation2d} for the Modules from the center of the robot.
   */
  public static final Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0), // Module 0
      new Translation2d(
          DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0), // Module 1
      new Translation2d(
          -DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0), // Module 2
      new Translation2d(
          -DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0), // Module 3
    };
  }

  // PID & FEEDFORWARD CONSTANTS
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

  // SIM CONSTANTS
  /** Moment of inertia value for Drive motor - Module wheel system in kilograms * meters squared */
  public static final double DRIVE_MOI_KG_M2 = 0.0003125;
  /** Moment of inertia value for Turn motor - Module wheel system in kilograms * meters squared */
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
}
