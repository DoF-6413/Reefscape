package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HeadingControllerConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

          /* If on the red alliance, rotate speeds by 180 degrees so that straight out from the red driver station is forward */
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  chassisSpeeds,
                  isRed ? drive.getRobotHeading().plus(Rotation2d.kPi) : drive.getRobotHeading()));
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
                      drive.getRobotHeading().getRadians(), headingSetpoint.get().getRadians());

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
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

              /* If on the red alliance, rotate speeds by 180 degrees so that straight out from the red driver station is forward */
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      chassisSpeeds,
                      isRed
                          ? drive.getRobotHeading().plus(Rotation2d.kPi)
                          : drive.getRobotHeading()));
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRobotHeading().getRadians()));
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
    final double RAMP_RATE_VOLTS_PER_SEC = 0.1;
    final double START_DELAY = 2;

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
            .withTimeout(START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * RAMP_RATE_VOLTS_PER_SEC;
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

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getDrivePositionRad();
                  state.lastAngle = drive.getRobotHeading();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRobotHeading();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      double[] positions = drive.getDrivePositionRad();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.DRIVETRAIN_RADIUS_M) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getDrivePositionRad();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.DRIVETRAIN_RADIUS_M) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
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
