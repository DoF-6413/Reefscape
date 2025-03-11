package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

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
      Pose2d elementPose, double wallDistanceMeters, double strafeOffsetMeters, boolean isFront) {
    var elementRotation = elementPose.getRotation();
    // double hypot =
    //     Math.hypot((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters, strafeOffsetMeters);
    // double hypotAngle =
    //     Math.atan2(strafeOffsetMeters, (DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters);
    // // Translated pose to send to Pathfinder, so that robot isn't commanded to go directly on top
    // of
    // // the specified field element's pose
    // var goalPose =
    //     new Pose2d(
    //         // Multiply the x by cos and y by sin of the field element angle so that the hypot
    //         // (field element to robot)
    //         // is the desired distance away from the field element
    //         elementPose.getX() + hypot * Math.cos(elementRotation.getRadians() - hypotAngle),
    //         elementPose.getY() + hypot * Math.sin(elementRotation.getRadians() - hypotAngle),
    //         // Rotate by 180 as the field elements' angles are rotated 180 degrees relative to
    // the
    //         // robot
    //         elementRotation.plus(isFront ? Rotation2d.k180deg : Rotation2d.kZero));

    var goalPose =
        new Pose2d(
            // Multiply the x by cos and y by sin of the field element angle so that the hypot
            // (field element to robot)
            // is the desired distance away from the field element
            elementPose.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters)
                    * elementRotation.getCos()
                + (strafeOffsetMeters * elementRotation.getSin()),
            elementPose.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters)
                    * elementRotation.getSin()
                + (strafeOffsetMeters * elementRotation.getCos()),
            // Rotate by 180 as the field elements' angles are rotated 180 degrees relative to the
            // robot
            elementRotation.plus(isFront ? Rotation2d.k180deg : Rotation2d.kZero));

    return AutoBuilder.pathfindToPoseFlipped(
        goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
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
  public static Command pathfindToCurrentTag(
      Drive drive, Vision vision, double wallDistanceMeters, BooleanSupplier stopTrigger) {
    return Commands.run(
        () -> {
          /*
           * Get ID of AprilTag currently seen by the front camera, if any. If an invalid ID is
           * given the apriltagPose Optional will be empty
           */
          var apriltagPose =
              FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(
                  vision.getTagID(VisionConstants.CAMERA.FRONT.CAMERA_INDEX));

          // If no valid tag returned then return a print messsage instead
          if (apriltagPose.isEmpty()) {
            Commands.print("Invalid AprilTag ID").until(stopTrigger).schedule();
          } else {

            // Pathfind to BRANCH pose. This method returns a command to pathfind to in front of the
            // BRANCH'S pose as to not drive into it.
            PathfindingCommands.pathfindToFieldElement(
                    apriltagPose.get().toPose2d(),
                    wallDistanceMeters,
                    PathPlannerConstants.ROBOT_MIDPOINT_TO_INTAKE,
                    true)
                .until(stopTrigger)
                .schedule();
          }
        },
        drive);
  }

  /**
   * Generates a trajectory for the robot to follow to the AprilTag corresponding to the ID inputed
   * with an additional distance translation. The trajectory will automatically be rotated to the
   * red alliance.
   *
   * @param tagID AprilTag ID of the desired AprilTag to align to.
   * @param wallDistanceMeters Distance in front of the AprilTag for the robot to end up.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the AprilTag.
   */
  public static Command pathfindToAprilTag(IntSupplier tagID, DoubleSupplier wallDistanceMeters) {
    // Get the 2d pose of the AprilTag associated with the inputed ID
    var apriltagPose =
        FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID.getAsInt()).get().toPose2d();
    /*
     * The goal pose is the end position for the center of the robot. Transforming by half the track
     * width will leave the robot right up against the tag and any additional distance can be added
     */
    var goalPose =
        new Pose2d(
            /*
             * Multiply the x by cos and y by sin of the tag angle so that the hypot (tag to robot)
             * is the desired distance away from the tag
             */
            apriltagPose.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters.getAsDouble())
                    * apriltagPose.getRotation().getCos(),
            apriltagPose.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + wallDistanceMeters.getAsDouble())
                    * apriltagPose.getRotation().getSin(),
            // Rotate by 180 as the AprilTag angles are rotated 180 degrees relative to the robot
            apriltagPose.getRotation().plus(Rotation2d.k180deg));

    return AutoBuilder.pathfindToPoseFlipped(
        goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }

  /**
   * Generates a trajectory for the robot to follow to a specified REEF BRANCH with an additional
   * distance translation. The trajectory will automatically be rotated to the red alliance.
   *
   * @param branchLetter Letter corresponding to BRANCH to pathfind to.
   * @param wallDistanceMeters Distance from the REEF wall in meters.
   * @return {@link Command} that makes the robot follow a trajectory to in front of the BRANCH.
   */
  public static Command pathfindToBranch(String branchLetter, double wallDistanceMeters) {
    // Position of BRANCH corresponding to zone the robot is in
    var branchPose = FieldConstants.BRANCH_POSES.get(branchLetter);

    if (branchLetter == null || branchLetter.isEmpty() || branchLetter.isBlank())
      return new InstantCommand();

    // Translated pose to send to Pathfinder, so that robot isn't commanded to go directly on top of
    // the BRANCH
    var goalPose =
        new Pose2d(
            // Multiply the x by cos and y by sin of the tag angle so that the hypot (tag to robot)
            // is the desired distance away from the tag
            branchPose.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2)
                        + FieldConstants.BRANCH_TO_WALL_X_M
                        + wallDistanceMeters)
                    * branchPose.getRotation().getCos(),
            branchPose.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2)
                        + FieldConstants.BRANCH_TO_WALL_X_M
                        + wallDistanceMeters)
                    * branchPose.getRotation().getSin(),
            // Rotate by 180 as the AprilTag angles are rotated 180 degrees relative to the robot
            branchPose.getRotation().plus(Rotation2d.k180deg));

    return AutoBuilder.pathfindToPoseFlipped(
        goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
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
  public static Command pathfindToClosestBranch(
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
          PathfindingCommands.pathfindToFieldElement(
                  FieldConstants.BRANCH_POSES.get(branchLetter),
                  wallDistanceMeters + FieldConstants.BRANCH_TO_WALL_X_M,
                  PathPlannerConstants.ROBOT_MIDPOINT_TO_INTAKE,
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
    String csLeft = RobotStateConstants.isRed() ? "CS2C" : "CS1C";
    String csRight = RobotStateConstants.isRed() ? "CS1C" : "CS2C";
    return Commands.run(
        () -> {
          if (drive.getCurrentPose2d().getY() > FieldConstants.FIELD_WIDTH / 2) {
            // Pathfind to the center of the CS to the left of the Driver Station
            PathfindingCommands.pathfindToFieldElement(
                    FieldConstants.CORAL_STATION_POSES.get(csLeft), wallDistanceMeters, 0, false)
                .until(stopTrigger)
                .schedule();
          } else {
            // Pathfind to the center of the CS to the right of the Driver Station
            PathfindingCommands.pathfindToFieldElement(
                    FieldConstants.CORAL_STATION_POSES.get(csRight), wallDistanceMeters, 0, false)
                .until(stopTrigger)
                .schedule();
          }
        },
        drive);
  }
}
