package frc.robot.Commands.TeleopCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/** The commands for on-the-fly trajectory following using PathPlanner's Pathfinding feature */
public class PathfindingCommands {
  /**
   * Generates a trajectory for the robot to follow to the best AprilTag seen. If no AprilTag is
   * seen, a message will be printed repeatedly to the console advising to change the robot mode to
   * move again.
   *
   * @param vision Vision subsystem
   * @param distanceFromTagMeters Distance in front of the AprilTag for the robot to end up
   * @return Command that makes the robot follow a trajectory to in front of the AprilTag
   */
  public static Command pathfindToCurrentTag(Vision vision, DoubleSupplier distanceFromTagMeters) {
    /**
     * Get ID of AprilTag currently seen by the front camera, if any. If an invalid ID is given the
     * apriltagPose Optional will be empty
     */
    var apriltagPose =
        FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(
            vision.getTagID(VisionConstants.CAMERA.FRONT.CAMERA_INDEX));

    // If no valid tag returned then return a print messsage instead
    if (apriltagPose.isEmpty()) return new PrintCommand("Invalid Tag ID");

    // Turn 3d AprilTag pose into a 2d pose
    var apriltagPose2d = apriltagPose.get().toPose2d();

    /**
     * The goal pose is the end position for the center of the robot. Transforming by half the track
     * width will leave the robot right up against the tag and any additional distance can be added
     */
    var goalPose =
        new Pose2d(
            /**
             * Multiply the x by cos and y by sin of the tag angle so that the hypot (tag to robot)
             * is the desired distance away from the tag
             */
            apriltagPose2d.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                    * apriltagPose2d.getRotation().getCos(),
            apriltagPose2d.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                    * apriltagPose2d.getRotation().getSin(),
            apriltagPose2d.getRotation().plus(Rotation2d.k180deg));

    // Rotate by 180 as the AprilTag angles are rotated 180 degrees relative to the robot
    return AutoBuilder.pathfindToPoseFlipped(
        goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }

  /**
   * Generates a trajectory for the robot to follow to the AprilTag corresponding to the ID inputed
   * with an additional distance translation
   *
   * @param tagID AprilTag ID of the desired AprilTag to align to
   * @param distanceFromTagMeters Distance in front of the AprilTag for the robot to end up
   * @return Command that makes the robot follow a trajectory to in front of the AprilTag
   */
  public static Command pathfindToAprilTag(
      IntSupplier tagID, DoubleSupplier distanceFromTagMeters) {
    // Get the 2d pose of the AprilTag associated with the inputed ID
    var apriltagPose =
        FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID.getAsInt()).get().toPose2d();
    /**
     * The goal pose is the end position for the center of the robot. Transforming by half the track
     * width will leave the robot right up against the tag and any additional distance can be added
     */
    var goalPose =
        new Pose2d(
            /**
             * Multiply the x by cos and y by sin of the tag angle so that the hypot (tag to robot)
             * is the desired distance away from the tag
             */
            apriltagPose.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                    * apriltagPose.getRotation().getCos(),
            apriltagPose.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                    * apriltagPose.getRotation().getSin(),
            // Rotate by 180 as the AprilTag angles are rotated 180 degrees relative to the robot
            apriltagPose.getRotation().plus(Rotation2d.k180deg));

    return AutoBuilder.pathfindToPoseFlipped(
        goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }

  public static Command pathfindToClosestReef(
      Drive drive, DoubleSupplier distanceFromBranchMeters) {
    var currentPose = drive.getCurrentPose2d();
    // Angle from REEF to robot
    double thetaDeg =
        Units.radiansToDegrees(
            Math.atan2(
                currentPose.getY() - FieldConstants.REEF_CENTER_TRANSLATION.getY(),
                currentPose.getX() - FieldConstants.REEF_CENTER_TRANSLATION.getX()));
    Commands.runOnce(() -> new PrintCommand("\n\n\n~~~~~THETA ANGLE: " + thetaDeg + "~~~~~\n\n\n"))
        .schedule();

    // Translated pose to send to Pathfinder, so that robot isn't commanded to go directly on top of
    // the BRANCH
    ////
    Pose2d goalPose;

    if (thetaDeg > 150 && thetaDeg < -150) {
      // In zone AB
      if (thetaDeg < 180 && thetaDeg > 0) {
        // Go to Left BRANCH (A)
        var branchPose = FieldConstants.BRANCH_POSES.get("A");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE A: " + goalPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (B)
        var branchPose = FieldConstants.BRANCH_POSES.get("B");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE B: " + goalPose.toString() + "\n\n\n"));
      }

    } else if (thetaDeg > -150 && thetaDeg < -90) {
      // In zone CD
      if (thetaDeg < -120) {
        // Go to Left BRANCH (C)
        // var branchPose = FieldConstants.BRANCH_POSES.get("C");
        // goalPose =
        //     new Pose2d(
        //         branchPose.getX()
        //             + (DriveConstants.TRACK_WIDTH_M / 2 + distanceFromBranchMeters.getAsDouble())
        //                 * Math.cos(branchPose.getRotation().getRadians()),
        //         branchPose.getY()
        //             + (DriveConstants.TRACK_WIDTH_M / 2 + distanceFromBranchMeters.getAsDouble())
        //                 * Math.sin(branchPose.getRotation().getRadians()),
        //         branchPose.getRotation().plus(Rotation2d.k180deg));

        var tagPose = FieldConstants.getAprilTagPose(17);
        var branchPose =
            new Pose2d(
                tagPose.get().toPose2d().getX()
                    - (Units.inchesToMeters(13) * Math.cos(Units.degreesToRadians(31.3286))),
                tagPose.get().toPose2d().getY()
                    - (Units.inchesToMeters(13) * Math.sin(Units.degreesToRadians(31.3286))),
                tagPose.get().toPose2d().getRotation());
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromBranchMeters.getAsDouble())
                        * tagPose.get().toPose2d().getRotation().getCos()),
                branchPose.getY()
                    + (((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromBranchMeters.getAsDouble())
                        * tagPose.get().toPose2d().getRotation().getSin()),
                tagPose.get().toPose2d().getRotation().plus(Rotation2d.k180deg));
        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE C: " + branchPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (D)
        var branchPose = FieldConstants.BRANCH_POSES.get("D");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (DriveConstants.TRACK_WIDTH_M / 2 + distanceFromBranchMeters.getAsDouble())
                        * Math.cos(-branchPose.getRotation().getRadians()),
                branchPose.getY()
                    + (DriveConstants.TRACK_WIDTH_M / 2 + distanceFromBranchMeters.getAsDouble())
                        * Math.sin(-branchPose.getRotation().getRadians()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE D: " + goalPose.toString() + "\n\n\n"));
      }
    } else if (thetaDeg > -90 && thetaDeg < -30) {
      // In zone EF
      if (thetaDeg < -60) {
        // Go to Left BRANCH (E)
        var branchPose = FieldConstants.BRANCH_POSES.get("E");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE E: " + goalPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (F)
        var branchPose = FieldConstants.BRANCH_POSES.get("F");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE F: " + goalPose.toString() + "\n\n\n"));
      }

    } else if (thetaDeg > -30 && thetaDeg < 30) {
      // In zone GH
      if (thetaDeg < 0) {
        // Go to Left BRANCH (G)
        var branchPose = FieldConstants.BRANCH_POSES.get("G");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE G: " + goalPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (H)
        var branchPose = FieldConstants.BRANCH_POSES.get("H");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE H: " + goalPose.toString() + "\n\n\n"));
      }

    } else if (thetaDeg > 30 && thetaDeg < 90) {
      // In zone IJ
      if (thetaDeg < 60) {
        // Go to Left BRANCH (I)
        var branchPose = FieldConstants.BRANCH_POSES.get("I");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE I: " + goalPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (J)
        var branchPose = FieldConstants.BRANCH_POSES.get("J");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE J: " + goalPose.toString() + "\n\n\n"));
      }

    } else {
      // In zone KL
      if (thetaDeg < 120) {
        // Go to Left BRANCH (K)
        var branchPose = FieldConstants.BRANCH_POSES.get("K");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE K: " + goalPose.toString() + "\n\n\n"));
      } else {
        // Go to Right BRANCH (L)
        var branchPose = FieldConstants.BRANCH_POSES.get("L");
        goalPose =
            new Pose2d(
                branchPose.getX()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getCos()),
                branchPose.getY()
                    + (distanceFromBranchMeters.getAsDouble() * branchPose.getRotation().getSin()),
                branchPose.getRotation().plus(Rotation2d.k180deg));

        return AutoBuilder.pathfindToPoseFlipped(
                goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
            .alongWith(new PrintCommand("\n\n\n GOAL POSE L: " + goalPose.toString() + "\n\n\n"));
      }
    }
  }
}
