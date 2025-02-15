package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HeadingControllerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** The commands to run the different driving modes and characterization routines of the robot */
public class DriveCommands {
  /**
   * Drives the robot with a 10% joystick deadband applied. This means joystick values between 0-0.1
   * (or 0-10%) will be ignored and not more the robot for both axises and rotation.
   *
   * <p>The joystick inputs run the robot at a percent scale from -1 (-100% reverse) to 1 (100%
   * forward)
   *
   * <p>Joystick inputs are based on the coordinate system of the field (+x along the length from
   * blue to red, +y along the height from red BARGE to blue BARGE). The robot's coordinate system
   * is 90 different compared to the field meaning its +x matches with the field +y and its +x
   * matches with the field +y. Due to this, the y-axis of the joystick is supplied to the xsupplier
   * and the x-axis is supplied to the y-supplier.
   *
   * @param drive Drivetrain subsystem
   * @param xVelocity Desired robot x-axis velocity from joystick y-axis input, percentage range
   *     from [-1, 1]
   * @param yVelocity Desired robot y-axis velocity from joystick x-axis input, percentage range
   *     from [-1, 1]
   * @param angularVelocity Desired angular velocity from joystick, percentage range from [-1, 1]
   */
  public static Command fieldRelativeDrive(
      Drive drive,
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      DoubleSupplier angularVelocity) {
    // Run modules based on field orientated Chassis speeds
    return Commands.run(
        () -> {
          // Calculate linear velocity from left joystick inputs
          Translation2d linearVelocity =
              getLinearVelocity(xVelocity.getAsDouble(), yVelocity.getAsDouble());
          // Calculate angular velocity from right joystick input
          double omega = getOmega(angularVelocity.getAsDouble());

          /* Calculate speed for the entire chassis based on the linear and angular velocities calculated from the joysticks */
          ChassisSpeeds chassisSpeeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S);

          // Determine alliance color
          boolean isRed =
              RobotStateConstants.getAlliance().isPresent()
                  && RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red;

          /* If on the red alliance, rotate speeds by 180 degrees so that straight out from the red driver station is forward */
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  chassisSpeeds,
                  isRed ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation()));
        },
        drive);
  }

  /**
   * Drives the robot without converting the speeds to the current heading of the robot. Forward
   * will ALWAYS be the front of the robot no matter the robot's heading
   *
   * @param drive Drive subsystem
   * @param xVelocity Desired robot x-axis velocity from joystick y-axis input, percentage range
   *     from [-1, 1]
   * @param yVelocity Desired robot y-axis velocity from joystick x-axis input, percentage range
   *     from [-1, 1]
   * @param angularVelocity Desired angular velocity from joystick, percentage range from [-1, 1]
   */
  public static Command robotRelativeDrive(
      Drive drive,
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      DoubleSupplier angularVelocity) {
    // Run modules based on field orientated Chassis speeds
    return Commands.run(
        () -> {
          // Calculate linear velocity from left joystick inputs
          Translation2d linearVelocity =
              getLinearVelocity(xVelocity.getAsDouble(), yVelocity.getAsDouble());
          // Calculate angular velocity from right joystick input
          double omega = getOmega(angularVelocity.getAsDouble());

          /*
           * Calculate speed for the entire chassis based on the linear and angular velocities calculated from the joysticks
           */
          drive.runVelocity(
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                  omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S));
        },
        drive);
  }

  /**
   * Drive the robot similar to field orientated drive however the robot will keep its heading
   * pointed towards the supplied heading
   *
   * @param drive Drive subsystem
   * @param xVelocity The desired robot x-axis velocity from joystick y-axis input, percentage range
   *     from [-1, 1]
   * @param yVelocity The desired robot y-axis velocity from joystick x-axis input, percentage range
   *     from [-1, 1]
   * @param headingSetpoint The desired heading for the robot
   */
  public static Command fieldRelativeDriveAtAngle(
      Drive drive,
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      Supplier<Rotation2d> headingSetpoint) {
    // Construct a PID contoller with trapezoidal motion profiling
    final ProfiledPIDController angleController =
        new ProfiledPIDController(
            HeadingControllerConstants.KP,
            0,
            HeadingControllerConstants.KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
                DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S));
    // Enable position wrapping for the PID controller since the robot rotation is as well
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Calculate linear velocity from left joystick inputs
              Translation2d linearVelocity =
                  getLinearVelocity(xVelocity.getAsDouble(), yVelocity.getAsDouble());
              // Calculate angular velocity from heading setpoint
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), headingSetpoint.get().getRadians());

              /*
               * Calculate speed for the entire chassis based on the linear and angular velocities calculated from the joysticks
               */
              ChassisSpeeds chassisSpeeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                      linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
                      omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S);

              // Determine alliance color
              boolean isRed =
                  RobotStateConstants.getAlliance().isPresent()
                      && RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red;

              /* If on the red alliance, rotate speeds by 180 degrees so that straight out from the red driver station is forward */
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      chassisSpeeds,
                      isRed ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation()));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the static and velocity feedforward constants for the Drive motors
   *
   * @param drive Drive subsystem
   * @return A command that slowly accelerates the Drive motors while collecting data (velocity and
   *     voltage)
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    double rampRateVoltPerSec = 0.1;
    double startDelay = 2;

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(startDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * rampRateVoltPerSec;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getAverageDriveVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Calculates the linear velocity from the left joystick inputs */
  private static Translation2d getLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /** Calculates the anglular velocity from the x-axix of the right joystick input */
  private static double getOmega(double omega) {
    omega = MathUtil.applyDeadband(omega, DriveConstants.DEADBAND);
    return Math.copySign(omega * omega, omega);
  }
}
