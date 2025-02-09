package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Utils.PathPlanner;
import java.util.function.IntSupplier;

public class PathfindingCommands {
  public PathfindingCommands() {}

  public static Command toAprilTag(PathPlanner pathPlanner, IntSupplier fiducialID) {
    if (fiducialID.getAsInt() < 1 || fiducialID.getAsInt() > 22) {
      return new PrintCommand("Invalid AprilTag ID: " + fiducialID.getAsInt());
    }

    var tagPose2d =
        VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(fiducialID.getAsInt()).get().toPose2d();
    var inFrontOfTag =
        new Pose2d(
            tagPose2d.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getCos(),
            tagPose2d.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getSin(),
            tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
    return pathPlanner.pathFindToPose(inFrontOfTag);
  }
}
