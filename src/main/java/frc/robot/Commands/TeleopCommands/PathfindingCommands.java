package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Utils.PathPlanner;
import java.util.function.DoubleSupplier;

public class PathfindingCommands {
  public PathfindingCommands() {}

  public static Command pathfindToAprilTag(Drive drive, PathPlanner pathPlanner, Vision vision) {
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

  public static Command pathPlannerPIDTuningTest(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle) {
    final double kMaxSpeedPerSecond = 2; // Meters per second
    final double kMaxSpeedPerSecondSquared =
        Math.pow(kMaxSpeedPerSecond, 2); // Same as above, but squared
    final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxSpeedPerSecond, kMaxSpeedPerSecondSquared);

    ProfiledPIDController snapController =
        new ProfiledPIDController(
            SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kP", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kI", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kD", 0),
            kThetaControllerConstraints);
    ProfiledPIDController xController =
        new ProfiledPIDController(
            SmartDashboard.getNumber("PIDFF/PathPlanner/X-kP", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/X-kI", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/X-kD", 0),
            kThetaControllerConstraints);
    ProfiledPIDController yController =
        new ProfiledPIDController(
            SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kP", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kI", 0),
            SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kD", 0),
            kThetaControllerConstraints);

    return Commands.run(
        () -> {
          xController.setPID(
              SmartDashboard.getNumber("PIDFF/PathPlanner/X-kP", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/X-kI", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/X-kD", 0));
          yController.setPID(
              SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kP", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kI", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/Y-kD", 0));
          snapController.setPID(
              SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kP", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kI", 0),
              SmartDashboard.getNumber("PIDFF/PathPlanner/Theta-kD", 0));

          xController.setGoal(new TrapezoidProfile.State(x.getAsDouble(), 0));
          yController.setGoal(new TrapezoidProfile.State(y.getAsDouble(), 0));
          snapController.setGoal(
              new TrapezoidProfile.State(Math.toRadians(angle.getAsDouble()), 0.0));

          double xAdjustment = xController.calculate(drive.getCurrentPose2d().getY());
          double yAdjustment = -yController.calculate(drive.getCurrentPose2d().getX());
          double angleAdjustment = snapController.calculate(drive.getRotation().getRadians());

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xAdjustment * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  yAdjustment * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  angleAdjustment * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
                  drive.getRotation()));
        });
  }
}
