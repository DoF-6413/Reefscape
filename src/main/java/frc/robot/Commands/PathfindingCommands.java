package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** The commands for on-the-fly trajectory following using PathPlanner's Pathfinding feature */
public class PathfindingCommands {
  /**
   * Generates a trajectory for the robot to follow to a specified field element with an additional
   * distance translation. The trajectory will automatically be rotated to the red alliance.
   *
   * @param elementPose {@link Pose2d} of the element to pathfind to.
   * @param wallDistanceMeters Distance from the field element in meters.
   * @param strafeOffsetMeters Left/right offset of the robot relative to the field element.
   *     Nesessary depending on mechanism in use
   * @param isFront {@code true} if to rotate goal pose by 180 for the front of the robot, {@code
   *     false} if to align with the back of the robot
   * @return {@link Command} that makes the robot follow a trajectory to in front of the field
   *     element.
   */
  public static Command pathfindToFieldElement(
      Drive drive,
      Pose2d elementPose,
      double wallDistanceMeters,
      double strafeOffsetMeters,
      boolean isFront) {
    var elementRotation = elementPose.getRotation();
    double hypot =
        Math.hypot((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters, strafeOffsetMeters);
    double hypotAngle =
        Math.atan2(strafeOffsetMeters, (DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters);
    // Translated pose to send to Pathfinder, so that robot isn't commanded to go directly on top
    // of the specified field element's pose
    var goalPose =
        new Pose2d(
            // Multiply the x by cos and y by sin of the field element angle so that the hypot
            // (field element to robot)
            // is the desired distance away from the field element
            elementPose.getX() + hypot * Math.cos(elementRotation.getRadians() + hypotAngle),
            elementPose.getY() + hypot * Math.sin(elementRotation.getRadians() + hypotAngle),
            // Rotate by 180 as the field elements' angles are rotated 180 degrees relative to
            // the
            // robot
            elementRotation.plus(isFront ? Rotation2d.k180deg : Rotation2d.kZero));

    return AutoBuilder.pathfindToPoseFlipped(
            goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
        .alongWith(
            Commands.runOnce(() -> Logger.recordOutput("Vision/Pathfinding/GoalPose", goalPose)));
  }

  /**
   * Generates a trajectory for the robot to follow to a specified field element with an additional
   * distance translation. The trajectory will automatically be rotated to the red alliance.
   *
   * @param drive {@link Drive} subsystem
   * @param elementPose {@link Pose2d} of the element to pathfind to.
   * @param wallDistanceMeters Distance from the field element in meters.
   * @param strafeOffsetMeters Left/right offset of the robot relative to the field element.
   *     Nesessary depending on mechanism in use
   * @param isFront {@code true} if to rotate goal pose by 180 for the front of the robot, {@code
   *     false} if to align with the back of the robot
   * @return {@link Command} that makes the robot follow a trajectory to in front of the field
   *     element.
   */
  public static DriveToPose driveToFieldElement(
      Drive drive,
      Pose2d elementPose,
      double wallDistanceMeters,
      double strafeOffsetMeters,
      boolean isFront) {
    // Calculate distance from the elements pose to the desired location of the center of the robot
    var elementRotation = elementPose.getRotation();
    double hypot =
        Math.hypot((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters, strafeOffsetMeters);
    double hypotAngle =
        Math.atan2(strafeOffsetMeters, (DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters);

    // Translated pose to send to Pathfinder, so that robot isn't commanded to go directly on top
    // of the specified field element's pose
    var goalPose =
        new Pose2d(
            // Multiply the x by cos and y by sin of the field element angle so that the hypot
            // (field element to robot)
            // is the desired distance away from the field element
            elementPose.getX() + hypot * Math.cos(elementRotation.getRadians() + hypotAngle),
            elementPose.getY() + hypot * Math.sin(elementRotation.getRadians() + hypotAngle),
            // Rotate by 180 as the field elements' angles are rotated 180 degrees relative to
            // the robot
            elementRotation.plus(isFront ? Rotation2d.k180deg : Rotation2d.kZero));

    return new DriveToPose(
        drive, () -> RobotStateConstants.isRed() ? FieldConstants.poseToRed(goalPose) : goalPose);
  }

  /**
   * Generates a trajectory for the robot to follow to the best AprilTag seen. If no AprilTag is
   * seen, a message will be printed repeatedly to the console advising to change the robot mode to
   * move again. The trajectory will automatically be rotated to the Red alliance.
   *
   * <p>Since a new trajectory is meant to be generated upon every button press, all the code must
   * be inside of the return. This is done by returning a {@code Commands.run()} with a block of
   * code inside of the lambda function for the {@link Runnable} parameter.
   *
   * @param drive {@link Drive} subsystem
   * @param vision {@link Vision} subsystem
   * @param wallDistanceMeters Distance in front of the AprilTag for the robot to end up.
   * @param stopTrigger {@link BooleanSupplier} with the condition to end the Pathfinding command.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the AprilTag.
   */
  // public static Command pathfindToCurrentTag(
  //     Drive drive, Vision vision, double wallDistanceMeters, BooleanSupplier stopTrigger) {
  //   return Commands.run(
  //       () -> {
  //         /*
  //          * Get ID of AprilTag currently seen by the front camera, if any. If an invalid ID is
  //          * given the apriltagPose Optional will be empty
  //          */
  //         var apriltagPose =
  //             FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(
  //                 vision.getTagID(VisionConstants.CAMERA.FRONT.CAMERA_INDEX));

  //         // If no valid tag returned then return a print messsage instead
  //         if (apriltagPose.isEmpty()) {
  //           Commands.print("Invalid AprilTag ID").until(stopTrigger).schedule();
  //         } else {

  //           // Pathfind to BRANCH pose. This method returns a command to pathfind to in front of
  // the
  //           // BRANCH'S pose as to not drive into it.
  //           PathfindingCommands.pathfindToFieldElement(
  //                   apriltagPose.get().toPose2d(),
  //                   wallDistanceMeters,
  //                   PathPlannerConstants.ROBOT_MIDPOINT_TO_SUPERSTRUCTURE,
  //                   true)
  //               .until(stopTrigger)
  //               .schedule();
  //         }
  //       },
  //       drive);
  // }

  /**
   * Generates a trajectory for the robot to follow to the AprilTag corresponding to the ID inputed
   * with an additional distance translation. The trajectory will automatically be rotated to the
   * red alliance.
   *
   * @param tagID AprilTag ID of the desired AprilTag to align to.
   * @param wallDistanceMeters Distance in front of the AprilTag for the robot to end up.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the AprilTag.
   */
  public static Command pathfindToAprilTag(
      Drive drive, int tagID, double wallDistanceMeters, boolean isFront) {
    // // Get the 2d pose of the AprilTag associated with the inputed ID
    // var apriltagPose = FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
    // /*
    //  * The goal pose is the end position for the center of the robot. Transforming by half the
    // track
    //  * width will leave the robot right up against the tag and any additional distance can be
    // added
    //  */
    // var goalPose =
    //     new Pose2d(
    //         /*
    //          * Multiply the x by cos and y by sin of the tag angle so that the hypot (tag to
    // robot)
    //          * is the desired distance away from the tag
    //          */
    //         apriltagPose.getX()
    //             + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters)
    //                 * apriltagPose.getRotation().getCos(),
    //         apriltagPose.getY()
    //             + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters)
    //                 * apriltagPose.getRotation().getSin(),
    //         // Rotate by 180 as the AprilTag angles are rotated 180 degrees relative to the robot
    //         apriltagPose.getRotation().plus(Rotation2d.k180deg));

    // return AutoBuilder.pathfindToPoseFlipped(
    //     goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
    return pathfindToFieldElement(
        drive,
        FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d(),
        wallDistanceMeters,
        Units.inchesToMeters(6),
        isFront);
  }

  /**
   * Generates a trajectory for the robot to follow to a specified REEF BRANCH with an additional
   * distance translation. The trajectory will automatically be rotated to the red alliance.
   *
   * @param branchLetter Letter corresponding to BRANCH to pathfind to.
   * @param wallDistanceMeters Distance from the REEF wall in meters.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the BRANCH.
   */
  public static Command pathfindToBranch(
      Drive drive, String branchLetter, double wallDistanceMeters) {
    return PathfindingCommands.pathfindToFieldElement(
        drive,
        FieldConstants.BRANCH_POSES.get(branchLetter),
        wallDistanceMeters + FieldConstants.BRANCH_TO_WALL_M,
        PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
        true);
  }

  /**
   * Generates a trajectory for the robot to follow to a specified REEF BRANCH with an additional
   * distance translation. The trajectory will automatically be rotated to the red alliance.
   *
   * @param drive {@link Drive} subsystem
   * @param branchLetter Letter corresponding to BRANCH to pathfind to.
   * @param wallDistanceMeters Distance from the REEF wall in meters.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the BRANCH.
   */
  public static DriveToPose driveToBranch(
      Drive drive, String branchLetter, double wallDistanceMeters) {
    return PathfindingCommands.driveToFieldElement(
        drive,
        FieldConstants.BRANCH_POSES.get(branchLetter),
        wallDistanceMeters + FieldConstants.BRANCH_TO_WALL_M,
        PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
        true);
  }

  /**
   * Generates a trajectory for the robot to follow to the nearest BRANCH. The trajectory will
   * automatically be rotated to the Red alliance.
   *
   * <p>Since a new trajectory is meant to be generated upon every button press, all the code must
   * be inside of the return. This is done by returning a {@code Commands.run()} with a block of
   * code inside of the lambda function for the {@link Runnable} parameter.
   *
   * @param drive {@link Drive} subsystem
   * @param wallDistanceMeters Distance from the REEF wall in meters.
   * @param stopTrigger {@link BooleanSupplier} with the condition to end the Pathfinding command.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the nearest
   *     BRANCH.
   */
  public static Command driveToClosestBranch(
      Drive drive, double wallDistanceMeters, BooleanSupplier stopTrigger) {

    return Commands.run(
        () -> {
          var currentPose = drive.getCurrentPose2d();
          // Angle from REEF to robot
          double thetaDeg =
              Units.radiansToDegrees(
                  Math.atan2(
                      currentPose.getY() - FieldConstants.REEF_CENTER_TRANSLATION.getY(),
                      currentPose.getX() - FieldConstants.REEF_CENTER_TRANSLATION.getX()));
          // Letter corresponding to BRANCH to pathfind to
          String branchLetter;

          // Decide which BRANCH to pathfind to
          if (thetaDeg > 150) {
            // BRANCH A (left)
            branchLetter = "A";
          } else if (thetaDeg < -150) {
            // BRANCH B (right)
            branchLetter = "B";
          } else if (thetaDeg < -90 && thetaDeg > -150) {
            // REEF zone CD
            if (thetaDeg < -120) {
              // BRANCH C (left)
              branchLetter = "C";
            } else {
              // BRANCH D (right)
              branchLetter = "D";
            }
          } else if (thetaDeg < -30 && thetaDeg > -90) {
            // REEF zone EF
            if (thetaDeg < -60) {
              // BRANCH E (left)
              branchLetter = "E";
            } else {
              // BRANCH F (right)
              branchLetter = "F";
            }
          } else if (thetaDeg < 30 && thetaDeg > -30) {
            // REEF zone GH
            if (thetaDeg < 0) {
              // BRANCH G (left)
              branchLetter = "G";
            } else {
              // BRANCH H (right)
              branchLetter = "H";
            }
          } else if (thetaDeg < 90 && thetaDeg > 30) {
            // REEF zone IJ
            if (thetaDeg < 60) {
              // BRANCH I (left)
              branchLetter = "I";
            } else {
              // BRANCH J (right)
              branchLetter = "J";
            }
          } else {
            // REEF zone KL
            if (thetaDeg < 120) {
              // BRANCH K (left)
              branchLetter = "K";
            } else {
              // BRANCH L (right)
              branchLetter = "L";
            }
          }

          // Pathfind to BRANCH pose. This method returns a command to pathfind to in front of the
          // BRANCH'S pose as to not drive into it.
          PathfindingCommands.driveToFieldElement(
                  drive,
                  FieldConstants.BRANCH_POSES.get(branchLetter),
                  wallDistanceMeters + FieldConstants.BRANCH_TO_WALL_M,
                  PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
                  true)
              .until(stopTrigger)
              .schedule();
        },
        drive);
  }

  /**
   * Generates a trajectory for the robot to follow to the nearest CORAL STATION. The trajectory
   * will automatically be rotated to the Red alliance.
   *
   * <p>Since a new trajectory is meant to be generated upon every button press, all the code must
   * be inside of the return. This is done by returning a {@code Commands.run()} with a block of
   * code inside of the lambda function for the {@link Runnable} parameter.
   *
   * @param drive {@link Drive} subsystem
   * @param wallDistanceMeters Distance from the CS wall in meters.
   * @param stopTrigger {@link BooleanSupplier} with the condition to end the Pathfinding command.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the nearest CS.
   */
  public static Command pathfindToClosestCoralStation(
      Drive drive, double wallDistanceMeters, BooleanSupplier stopTrigger) {
    // Initialize CORAL STATIONS based on alliance color
    String csLeft = RobotStateConstants.isRed() ? "CS2L" : "CS1R";
    String csRight = RobotStateConstants.isRed() ? "CS1R" : "CS2L";
    return Commands.run(
        () -> {
          if (drive.getCurrentPose2d().getY() > FieldConstants.FIELD_WIDTH / 2) {
            // Pathfind to the center of the CS to the left of the Driver Station
            PathfindingCommands.pathfindToFieldElement(
                    drive,
                    FieldConstants.CORAL_STATION_POSES.get(csLeft),
                    wallDistanceMeters,
                    -PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
                    false)
                .until(stopTrigger)
                .schedule();
          } else {
            // Pathfind to the center of the CS to the right of the Driver Station
            PathfindingCommands.pathfindToFieldElement(
                    drive,
                    FieldConstants.CORAL_STATION_POSES.get(csRight),
                    wallDistanceMeters,
                    -PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
                    false)
                .until(stopTrigger)
                .schedule();
          }
        },
        drive);
  }

  /**
   * Generates a trajectory for the robot to follow to the nearest CORAL STATION. The trajectory
   * will automatically be rotated to the Red alliance.
   *
   * <p>Since a new trajectory is meant to be generated upon every button press, all the code must
   * be inside of the return. This is done by returning a {@code Commands.run()} with a block of
   * code inside of the lambda function for the {@link Runnable} parameter.
   *
   * @param drive {@link Drive} subsystem
   * @param wallDistanceMeters Distance from the CS wall in meters.
   * @param stopTrigger {@link BooleanSupplier} with the condition to end the Pathfinding command.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the nearest CS.
   */
  public static Command driveToClosestCoralStation(
      Drive drive, double wallDistanceMeters, BooleanSupplier stopTrigger) {
    // Initialize CORAL STATIONS based on alliance color
    String csLeft = RobotStateConstants.isRed() ? "CS2L" : "CS1R";
    String csRight = RobotStateConstants.isRed() ? "CS1R" : "CS2L";

    return Commands.run(
        () -> {
          if (drive.getCurrentPose2d().getY() > FieldConstants.FIELD_WIDTH / 2) {
            // Pathfind to the center of the CS to the left of the Driver Station
            PathfindingCommands.driveToFieldElement(
                    drive,
                    FieldConstants.CORAL_STATION_POSES.get(csLeft),
                    wallDistanceMeters,
                    -PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
                    false)
                .until(stopTrigger)
                .schedule();
          } else {
            // Pathfind to the center of the CS to the right of the Driver Station
            PathfindingCommands.driveToFieldElement(
                    drive,
                    FieldConstants.CORAL_STATION_POSES.get(csRight),
                    wallDistanceMeters,
                    PathPlannerConstants.SUPERSTRUCTURE_OFFSET,
                    false)
                .until(stopTrigger)
                .schedule();
          }
        },
        drive);
  }

  public static Command alignToBranch(Drive drive, int reefTag, String branch) {
    // return PathfindingCommands.pathfindToAprilTag(drive, reefTag, 1, true)
    //     .andThen(
    //         Commands.waitUntil(
    //             () ->
    //                 drive.getChassisSpeeds().vxMetersPerSecond < 0.25
    //                     && drive.getChassisSpeeds().vyMetersPerSecond < 0.25))
    //     .andThen(
    return PathfindingCommands.driveToBranch(drive, branch, 0)
        .alongWith(
            Commands.repeatingSequence(
                Commands.print("Precision Alignment").alongWith(Commands.waitSeconds(0.25))));
  }
}
