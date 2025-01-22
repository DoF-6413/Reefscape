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
  public static final double DRIVE_GEAR_RATIO = 6.12;
  /** Gear Ratio for MK4i Neos */
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double DRIVE_KP = 0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static final double DRIVE_KI = 0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static final double DRIVE_KD = 0;
  /** KS represents the voltage required to overcome static friction */
  public static final double DRIVE_KS_KRAKEN = 0.115;
  /** KV represents the voltage used every second per meter */
  public static final double DRIVE_KV_KRAKEN = 0.12978;

  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static final double TURN_KP = 6.4;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static final double TURN_KI = 0.0;
  /** KD represents the constant multiplied by the change in error over time (Derivative Error) */
  public static final double TURN_KD = 0.05;

  /** Max Linear Speed of Robot */
  public static final double MAX_LINEAR_SPEED_M_PER_S =
      5.2; // TODO: Update? Since robot is larger //TODO:I still not knowing the answere
  /** Set the inverted for the turn SparkMax */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_S =
      MAX_LINEAR_SPEED_M_PER_S / (Math.sqrt(2) * TRACK_WIDTH_M / 2);

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
  // TODO: Update

  public static final double DRIVE_MOI_KG_M2 = 0.0;

  public static final double TURN_MOI_KG_M2 = 0.0;
  
  /** Translation 2d assumes that the robot front facing is in the positive x direction and the
  * robot left is in the positive y direction <br></br>

  * Directions of the VECTORS that each module turns in (i.e. Module 0, 
  * front right, when the robot spins ccw, will point towards Quadrant II
  * which is (-,+)) */
  public static final Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, -DriveConstants.TRACK_WIDTH_M / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_M / 2.0, DriveConstants.TRACK_WIDTH_M / 2.0),
    };
  }
  // Krakens, Neos, CANcoders
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
    FRONT_RIGHT(1.8775924843720249), // Module 0
    FRONT_LEFT(1.1489516101263453), // Module 1
    BACK_LEFT(-0.6534758156392831), // Module 2
    BACK_RIGHT(1.0615147052168636); // Module 3

    public final double OFFSET;

    ABSOLUTE_ENCODER_OFFSET(double value) {
      OFFSET = value;
    }
  }
}
