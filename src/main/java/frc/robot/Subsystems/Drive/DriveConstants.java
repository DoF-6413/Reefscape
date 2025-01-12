// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  // REAL CONSTANTS
  /** Radius of the wheel in meters */
  public static final double WHEEL_RADIUS_M = Units.inchesToMeters(2);
  /** Side length of the robot in meters */
  public static final double TRACK_WIDTH_M = Units.inchesToMeters(29);
  /** Radius of the robot (diagonal) in meters */
  public static final double DRIVETRAIN_RADIUS_M = TRACK_WIDTH_M / 2 * Math.sqrt(2);
  /** Gear Ratio for MK4i L3 Krakens */
  public static final double GEAR_RATIO = 6.12;
  /** Max Linear Speed of Robot */
  public static final double MAX_LINEAR_SPEED_M_PER_S = 5.2; // TODO: Update? Since robot is larger
  /** Set the inverted for the turn SparkMax */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_S =
      MAX_LINEAR_SPEED_M_PER_S / (Math.sqrt(2) * TRACK_WIDTH_M / 2);

  public static final boolean TURN_IS_INVERTED = true;
  /** */
  public static final double UPDATE_FREQUENCY_HZ = 100;
  /** Current limiting in amps */
  public static final int CUR_LIM_A = 60;
  /** Enables the current limit */
  public static final boolean ENABLE_CUR_LIM = true;
  /**
   * Within 5% of the desired direction, the joystick is considered to be going in that direction
   */
  public static final double DEADBAND = 0.05;

  // SIM CONSTANTS
  // TODO: Update

  /** Max Linear Speed of Robot */
  public static final double DRIVE_MOI_KG_M2 = 0.0;

  public static final double TURN_MOI_KG_M2 = 0.0;

  public static final Translation2d[] getModuleTranslations() {
    // Translation 2d assumes that the robot front facing is in the positive x direction and the
    // robot left is in the positive y direction
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
    };
  }

  // TODO: update all CAN IDs
  public enum DRIVE_MOTOR {
    FRONT_RIGHT(0), // Module 0
    FRONT_LEFT(0), // Module 1
    BACK_LEFT(0), // Module 2
    BACK_RIGHT(0); // Module 3

    public final int CAN_ID;

    DRIVE_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  public enum TURN_MOTOR {
    FRONT_RIGHT(0), // Module 0
    FRONT_LEFT(0), // Module 1
    BACK_LEFT(0), // Module 2
    BACK_RIGHT(0); // Module 3

    public final int CAN_ID;

    TURN_MOTOR(int value) {
      CAN_ID = value;
    }
  }

  public enum ABSOLUTE_ENCODER {
    FRONT_RIGHT(0), // Module 0
    FRONT_LEFT(0), // Module 1
    BACK_LEFT(0), // Module 2
    BACK_RIGHT(0); // Module 3

    public final int CAN_ID;

    ABSOLUTE_ENCODER(int value) {
      CAN_ID = value;
    }
  }

  public enum ABSOLUTE_ENCODER_OFFSET {
    FRONT_RIGHT(0), // Module 0
    FRONT_LEFT(0), // Module 1
    BACK_LEFT(0), // Module 2
    BACK_RIGHT(0); // Module 3

    public final int OFFSET;

    ABSOLUTE_ENCODER_OFFSET(int value) {
      OFFSET = value;
    }
  }
}
