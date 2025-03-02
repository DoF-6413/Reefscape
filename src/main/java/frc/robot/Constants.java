// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class RobotStateConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    /**
     * @return Robot Mode (Real/Sim/Replay)
     */
    public static Mode getMode() {
      if (RobotBase.isReal()) {
        return Mode.REAL;
      } else if (RobotBase.isSimulation()) {
        return Mode.SIM;
      } else {
        return Mode.REPLAY;
      }
    }

    /**
     * @return Alliance from FMS
     */
    public static Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    /** Whether or not the robot is on the Red Alliance */
    public static boolean isRed() {
      return RobotStateConstants.getAlliance().isPresent()
          && RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red;
    }

    /** After 500 seconds, the CAN times out */
    public static final int CAN_CONFIG_TIMEOUT_SEC = 500;

    /** Every 20 ms, periodic commands loop */
    public static final double LOOP_PERIODIC_SEC = 0.02;

    /** Max voltage to send to motor */
    public static final double MAX_VOLTAGE = 12;

    /** Weight of the robot with bumpers and battery */
    public static final double ROBOT_WEIGHT_KG = Units.lbsToKilograms(135);
    /** Weight of the Proto-Bot with battery */
    public static final double PROTOBOT_WEIGHT_KG = Units.lbsToKilograms(35);
    /** Rough moment of inertia calculation of the robot in kilograms * meters squared */
    public static final double ROBOT_MOI_KG_M2 =
        (1.0 / 12.0)
            * ROBOT_WEIGHT_KG
            * ((DriveConstants.TRACK_WIDTH_M * DriveConstants.TRACK_WIDTH_M)
                + (DriveConstants.TRACK_WIDTH_M * DriveConstants.TRACK_WIDTH_M));
    /** Rough moment of inertia calculation of the robot in kilograms * meters squared */
    public static final double PROTOBOT_MOI_KG_M2 =
        (1.0 / 12.0)
            * PROTOBOT_WEIGHT_KG
            * ((DriveConstants.TRACK_WIDTH_M * DriveConstants.TRACK_WIDTH_M)
                + (DriveConstants.TRACK_WIDTH_M * DriveConstants.TRACK_WIDTH_M));
  }

  /** Controller ports */
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int AUX_CONTROLLER = 1;
  }

  /** Heading Controller */
  public static class HeadingControllerConstants {
    public static final double KP = 1.0;
    public static final double KD = 0.0;
  }

  /** Field measurements */
  public final class FieldConstants {
    /** 3d field setup with the locations of the AprilTags loaded from WPILib JSON files */
    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags(),
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldLength(),
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth());
    /** Field length of the Welded Reefscape field */
    public static final double FIELD_LENGTH = APRILTAG_FIELD_LAYOUT.getFieldLength();
    /** Field width of the Welded Reefscape field */
    public static final double FIELD_WIDTH = APRILTAG_FIELD_LAYOUT.getFieldWidth();
    /**
     * The 3d pose is an optinal. If an ID outside of the range of [1, 22] then the Optional value
     * returned will be null
     *
     * @param ID Number corresponding to the ID of the desired AprilTag
     * @return An optional value containing the 3d pose of an AprilTag
     */
    public static Optional<Pose3d> getAprilTagPose(int ID) {
      return APRILTAG_FIELD_LAYOUT.getTagPose(ID);
    }

    /**
     * Translation of the center of the REEF from the origin point (bottom left corner) of the
     * field. Measured in meters
     */
    public static final Translation2d REEF_CENTER_TRANSLATION =
        new Translation2d(
            RobotStateConstants.isRed()
                ? FIELD_LENGTH - Units.inchesToMeters(176.746)
                : Units.inchesToMeters(176.746),
            FIELD_WIDTH / 2.0);

    /** A Map that links the BRANCH letter to its position on the field as a {@link Pose2d} */
    public static final Map<String, Pose2d> BRANCH_POSES = new HashMap<>();
    /**
     * The center of each face of the REEF, aka where the AprilTag is located. Definded starting at
     * the inner face (facing towards opposite alliance side) in clockwise order
     */
    public static final Pose2d[] CENTER_FACES = new Pose2d[6];
    /** Distance from the BRANCH to the REEF face wall in meters */
    public static final double BRANCH_TO_WALL_X_M = Units.inchesToMeters(7);

    /** A Map that links the CORAL STATION names to its position on the field as a {@link Pose2d} */
    public static final Map<String, Pose2d> CORAL_STATION_POSES = new HashMap<>();
    /**
     * {@link Pose2d} of the center of the CORAL STATIONS (same as the AprilTag pose). Bottom left
     * is index 0, top left is index 1
     */
    public static final Pose2d[] CENTER_CORAL_STATION = new Pose2d[2];

    static {
      // Initialize faces starting from inner face and in clockwise order
      CENTER_FACES[0] = getAprilTagPose(21).get().toPose2d();
      CENTER_FACES[1] = getAprilTagPose(22).get().toPose2d();
      CENTER_FACES[2] = getAprilTagPose(17).get().toPose2d();
      CENTER_FACES[3] = getAprilTagPose(18).get().toPose2d();
      CENTER_FACES[4] = getAprilTagPose(19).get().toPose2d();
      CENTER_FACES[5] = getAprilTagPose(20).get().toPose2d();
      /**
       * Letters of BRANCHES in same order as faces, first 6 are left BRANCHES, last 6 are right
       * BRANCHES
       */
      String BRANCH_LETTERS = "GECAKIHFDBLJ";
      /** Hypotenuse from AprilTag to BRANCH */
      double ARPILTAG_TO_BRANCH_HYPOT_M = Units.inchesToMeters(13);
      /** Angle from AprilTag to BRANCH that the hypotenuse makes */
      double ARPILTAG_TO_BRANCH_ANGLE_RAD = Units.degreesToRadians(30);

      // Initialize BRANCH poses
      for (int i = 0; i < 6; i++) {
        // Left BRANCH of REEF face
        var leftBranch =
            new Pose2d(
                CENTER_FACES[i].getX()
                    + (-ARPILTAG_TO_BRANCH_HYPOT_M
                        * Math.cos(
                            ARPILTAG_TO_BRANCH_ANGLE_RAD
                                + CENTER_FACES[i].getRotation().getRadians())),
                CENTER_FACES[i].getY()
                    + (-ARPILTAG_TO_BRANCH_HYPOT_M
                        * Math.sin(
                            ARPILTAG_TO_BRANCH_ANGLE_RAD
                                + CENTER_FACES[i].getRotation().getRadians())),
                CENTER_FACES[i].getRotation());

        // Right BRANCH of REEF face
        var rightBranch =
            new Pose2d(
                CENTER_FACES[i].getX()
                    + (-ARPILTAG_TO_BRANCH_HYPOT_M
                        * Math.cos(
                            -ARPILTAG_TO_BRANCH_ANGLE_RAD
                                + CENTER_FACES[i].getRotation().getRadians())),
                CENTER_FACES[i].getY()
                    + (-ARPILTAG_TO_BRANCH_HYPOT_M
                        * Math.sin(
                            -ARPILTAG_TO_BRANCH_ANGLE_RAD
                                + CENTER_FACES[i].getRotation().getRadians())),
                CENTER_FACES[i].getRotation());

        // Map poses to corresponding BRANCH letter
        BRANCH_POSES.put(BRANCH_LETTERS.substring(i, i + 1), leftBranch);
        BRANCH_POSES.put(BRANCH_LETTERS.substring(i + 6, i + 7), rightBranch);
      }
      // Initialize the locations of the center of the CORAL STATIONS
      CENTER_CORAL_STATION[0] = APRILTAG_FIELD_LAYOUT.getTagPose(13).get().toPose2d();
      CENTER_CORAL_STATION[1] = APRILTAG_FIELD_LAYOUT.getTagPose(12).get().toPose2d();
      /**
       * Distance from the center of the CS to the left/right sides. 76 = CS Width, (76 in / 2) -
       * (76 in / 6) = 25.3333 in
       */
      double CENTER_TO_SIDE = Units.inchesToMeters(25.3333);
      for (int i = 0; i < 2; i++) {
        // Left CORAL STATION area
        var leftCS =
            new Pose2d(
                CENTER_CORAL_STATION[i].getX()
                    + (CENTER_TO_SIDE
                        * Math.cos(
                            -Math.PI / 2 + CENTER_CORAL_STATION[i].getRotation().getRadians())),
                CENTER_CORAL_STATION[i].getY()
                    + (CENTER_TO_SIDE
                        * Math.sin(
                            -Math.PI / 2 + CENTER_CORAL_STATION[i].getRotation().getRadians())),
                CENTER_CORAL_STATION[i].getRotation());
        // Right CORAL STATION area
        var rightCS =
            new Pose2d(
                CENTER_CORAL_STATION[i].getX()
                    + (-CENTER_TO_SIDE
                        * Math.cos(
                            -Math.PI / 2 + CENTER_CORAL_STATION[i].getRotation().getRadians())),
                CENTER_CORAL_STATION[i].getY()
                    + (-CENTER_TO_SIDE
                        * Math.sin(
                            -Math.PI / 2 + CENTER_CORAL_STATION[i].getRotation().getRadians())),
                CENTER_CORAL_STATION[i].getRotation());

        // Map poses to names
        CORAL_STATION_POSES.put("CS" + (i + 1) + "C", CENTER_CORAL_STATION[i]);
        CORAL_STATION_POSES.put("CS" + (i + 1) + "L", leftCS);
        CORAL_STATION_POSES.put("CS" + (i + 1) + "R", rightCS);
      }
    }
  }

  /** Constants for PathPlanner configurations and Pathfinding */
  public final class PathPlannerConstants {
    /* Configuration */
    // PID
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KD = 0.0;
    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KD = 0.0;
    /** Coefficient of friction between wheels and the carpet */
    public static final double WHEEL_FRICTION_COEFF = 0.7;
    /** Swerve Module configuartion for PathPlanner */
    public static final ModuleConfig MODULE_CONFIG =
        new ModuleConfig(
            DriveConstants.WHEEL_RADIUS_M,
            DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            PathPlannerConstants.WHEEL_FRICTION_COEFF,
            DCMotor.getKrakenX60(1),
            DriveConstants.DRIVE_GEAR_RATIO,
            DriveConstants.CUR_LIM_A,
            1);
    /** Robot configuarion for PathPlanner */
    public static final RobotConfig ROBOT_CONFIG =
        new RobotConfig(
            RobotStateConstants.PROTOBOT_WEIGHT_KG,
            RobotStateConstants.PROTOBOT_MOI_KG_M2,
            MODULE_CONFIG,
            DriveConstants.getModuleTranslations());

    /* Pathfinding */
    /** Max translational and rotational velocity and acceleration used for Pathfinding */
    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(5, 5, Units.degreesToRadians(515.65), Units.degreesToRadians(262.82));
    /** Default distance away from any wall when the robot is Pathfinding towards one */
    public static final double DEFAULT_WALL_DISTANCE_M = Units.inchesToMeters(12);
  }
}
