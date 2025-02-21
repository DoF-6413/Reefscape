package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
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
    return AutoBuilder.pathfindToPose(goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
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

    return AutoBuilder.pathfindToPose(goalPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }
}
