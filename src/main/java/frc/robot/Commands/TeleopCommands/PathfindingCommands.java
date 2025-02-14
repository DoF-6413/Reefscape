package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Utils.PathPlanner;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/** The commands for on-the-fly trajectory following using PathPlanner's Pathfinding feature */
public class PathfindingCommands {
  /**
   * Generates a trajectory for the robot to follow to the best AprilTag seen. If no AprilTag is
   * seen, a message will be printed repeatedly to the console advising to change the robot mode to
   * move again.
   *
   * @param drive Drivetrain subsystem
   * @param pathplanner PathPlanner for its Pathfinding utility to generate a trajectory and run
   *     robot to follow it
   * @param vision Vision subsystem
   */
  public static Command pathfindToCurrentTag(Drive drive, Vision vision, PathPlanner pathplanner) {
    return Commands.run(
        () -> {
          var goalPose =
              VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(
                  vision.getTagID(VisionConstants.CAMERA.FRONT.CAMERA_INDEX));

          if (goalPose.isEmpty()) {
            new PrintCommand("Invalid Tag ID \nSwitch Drive mode to drive (press Y)").schedule();

          } else {
            var goalPose2d = goalPose.get().toPose2d();
            var targetPose =
                new Pose2d(
                    goalPose2d.getX()
                        + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                            * goalPose2d.getRotation().getCos(),
                    goalPose2d.getY()
                        + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                            * goalPose2d.getRotation().getSin(),
                    goalPose2d.getRotation().plus(Rotation2d.k180deg));

            pathplanner.pathFindToPose(targetPose).schedule();
          }
        },
        drive);
  }

  /**
   * Generates a trajectory for the robot to follow to the AprilTag corresponding to the ID inputed
   *
   * @param drive Drivetrain subsystem
   * @param pathplanner PathPlanner for its Pathfinding utility
   * @param tagID AprilTag ID of the desired AprilTag to align to
   */
  public static Command pathfindToAprilTag(
      Drive drive,
      PathPlanner pathplanner,
      IntSupplier tagID,
      DoubleSupplier distanceFromTagMeters,
      BooleanSupplier stopTrigger) {
    return Commands.run(
        () -> {
          var goalPose =
              VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID.getAsInt()).get().toPose2d();

          var targetPose =
              new Pose2d(
                  goalPose.getX()
                      + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                          * goalPose.getRotation().getCos(),
                  goalPose.getY()
                      + ((DriveConstants.TRACK_WIDTH_M / 2) + distanceFromTagMeters.getAsDouble())
                          * goalPose.getRotation().getSin(),
                  goalPose.getRotation().plus(Rotation2d.k180deg));

          pathplanner.pathFindToPose(targetPose).until(stopTrigger).schedule();
        },
        drive);
  }
}
