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
import java.util.function.IntSupplier;

public class PathfindingCommands {
  public PathfindingCommands() {}

  public static Command pathfindToCurrentTag(Drive drive, PathPlanner pathPlanner, Vision vision) {
    return Commands.run(
        () -> {
          var goalPose = VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(vision.getTagID());

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

            pathPlanner.pathFindToPose(targetPose).schedule();
          }
        },
        drive);
  }

  public static Command pathfindToAprilTag(
      Drive drive, PathPlanner pathPlanner, IntSupplier tagID) {
    return Commands.run(
        () -> {
          var goalPose =
              VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagID.getAsInt()).get().toPose2d();

          var targetPose =
              new Pose2d(
                  goalPose.getX()
                      + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                          * goalPose.getRotation().getCos(),
                  goalPose.getY()
                      + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                          * goalPose.getRotation().getSin(),
                  goalPose.getRotation().plus(Rotation2d.k180deg));

          pathPlanner.pathFindToPose(targetPose).schedule();
        },
        drive);
  }
}
