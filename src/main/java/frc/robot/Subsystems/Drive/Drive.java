package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Gyro.Gyro;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final Module[] modules = new Module[4];
  private final Gyro gyro;
  private Twist2d twist = new Twist2d();

  // The swerve drive kinematics
  public SwerveDriveKinematics swerveDriveKinematics;

  // The current state of the robot
  public ChassisSpeeds setpoint = new ChassisSpeeds();

  // Gets previous gyro yaw
  public Rotation2d lastGyroYaw = new Rotation2d();

  // Gets previous module positions
  private double[] lastModulePositionsMeters;

  public Drive(
      ModuleIO FRModuleIO,
      ModuleIO FLModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      Gyro gyro) {

    // Initialize the Drive subsystem
    System.out.println("[Init] Creating Drive");
    this.gyro = gyro;
    modules[0] = new Module(FRModuleIO, 0);
    modules[1] = new Module(FLModuleIO, 1);
    modules[2] = new Module(BLModuleIO, 2);
    modules[3] = new Module(BRModuleIO, 3);

    // Initialize the swerve drive kinematics
    swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // creates four modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
  }

  /**
   * Sets the entire Drive Train to either brake or coast mode
   *
   * @param isDisabled True for brake, false for coast
   */
  public void setBrakeModeAll(boolean isDisabled) {
    for (var module : modules) {
      module.setBrakeMode(isDisabled ? true : false);
    }
  }

  /**
   * Sets the Velocity of the Swerve Drive through Passing in a ChassisSpeeds (Can be Field Relative
   * OR Robot Orientated)
   *
   * @param speeds the desired chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    setpoint = discreteSpeeds;

    SwerveModuleState[] setpointStates = swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_S);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisStates/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drivetrain with raw values on a scale
   *
   * @param x velociy in x direction of Entire Swerve Drive
   * @param y velocity in y direction of Entire Swerve Drive
   * @param rot Angular Velocity of Entire Swerve Drive
   */
  public void setRaw(double x, double y, double rot) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, this.getRotation()));
  }

  /**
   * @return A list of SwerveModulePositions containing the change in module position and angle
   */
  public SwerveModulePosition[] getWheelDeltas() {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    /* Wheel Deltas or Wheel Positions */
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters()
                  - lastModulePositionsMeters[i]), // This calculates the change in angle
              modules[i].getAngle()); // Gets individual MODULE rotation
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }
    return wheelDeltas;
  }

  public Rotation2d getRotation() {

    var gyroYaw = new Rotation2d(gyro.getYaw().getRadians());

    /*
     * Twist2d is a change in distance along an arc
     * // x is the forward distance driven
     * // y is the distance driven to the side
     * // (left positive), and the component is the change in heading.
     */
    if (gyro.isConnected()) {
      twist =
          new Twist2d(
              twist.dx,
              twist.dy,
              gyroYaw.minus(lastGyroYaw).getRadians()); // Updates twist based on GYRO
    } else {
      twist =
          swerveDriveKinematics.toTwist2d(
              getWheelDeltas()); // Updates Twist Based on MODULE position
      gyroYaw =
          lastGyroYaw.minus(
              new Rotation2d(twist.dtheta)); // Updates rotation 2d based on robot module position
    }
    lastGyroYaw = gyroYaw;
    return lastGyroYaw;
  }

  public void driveWithDeadband(double x, double y, double rot) {
    // apply deadband to x, y, and rot

    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);

    Rotation2d linearDirection = new Rotation2d();
    if (x != 0 && y != 0) {
      linearDirection = new Rotation2d(x, y);
    }
    double omega = MathUtil.applyDeadband(rot, DriveConstants.DEADBAND);

    // square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calculate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // the actual run command itself
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
            this.getRotation()));
  }
}
