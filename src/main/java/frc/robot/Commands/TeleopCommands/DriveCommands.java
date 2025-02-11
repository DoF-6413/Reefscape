package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import frc.robot.Utils.PathPlanner;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

  public DriveCommands() {}

  /**
   * Drives the robot with a 10% joystick deadband applied. This means joystick values between 0-0.1
   * (or 0-10%) will be ignored and not more the robot for both axises and rotation.
   *
   * <p>The joystick inputs run the robot at a percent scale from -1 (-100% reverse) to 1 (100%
   * forward)
   *
   * @param xSupplier The desired x-axis velocity from joystick
   * @param ySupplier The desired y-axis velocity from joystick
   * @param omegaSupplier The desired angular velocity from joystick
   */
  public static Command fieldRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Run modules based on field orientated Chassis speeds
    return Commands.run(
        () -> {
          double omega = getOmega(omegaSupplier.getAsDouble());
          Translation2d linearVelocity =
              getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
                  drive.getRotation()));
        },
        drive);
  }

  public static Command robotRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Run modules based on field orientated Chassis speeds
    return Commands.run(
        () -> {
          double omega = getOmega(omegaSupplier.getAsDouble());
          Translation2d linearVelocity =
              getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          drive.runVelocity(
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S));
        },
        drive);
  }

  public static Command fieldRelativeDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            0.1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
                DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              Translation2d linearVelocity =
                  getLinearVelocity(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              drive.runVelocity(
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                      linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                      omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command alignToPose(Drive drive, PathPlanner pathPlanner, Vision vision) {
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

  private static double getOmega(double omega) {
    omega = MathUtil.applyDeadband(omega, DriveConstants.DEADBAND);
    return Math.copySign(omega * omega, omega);
  }

  private static Translation2d getLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    // NOTE: The x & y values range from -1 to +1, so their squares are as well
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
