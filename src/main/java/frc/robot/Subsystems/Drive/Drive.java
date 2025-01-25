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

  private final Module[] m_modules = new Module[4];
  private final Gyro m_gyro;
  private Twist2d m_twist = new Twist2d();

  // The swerve drive kinematics
  public SwerveDriveKinematics m_swerveDriveKinematics;

  // Previous yaw angle of the robot
  public Rotation2d m_lastRobotYaw = new Rotation2d();

  // Gets previous module positions
  private double[] m_lastModulePositionsMeters = new double[4];

  /**
   * Constructs a new Drive subsystem instance.
   *
   * <p>This constructor creates a new Drive object that stores the IO implementation of each Module
   * and the Gyroscope.
   *
   * @param FRModuleIO Front Right ModuleIO implementation of the current robot mode
   * @param FLModuleIO Front Left ModuleIO implementation of the current robot mode
   * @param BLModuleIO Back Left ModuleIO implementation of the current robot mode
   * @param BRModuleIO Back Right ModuleIO implementation of the current robot mode
   * @param gyro Gyro subsystem
   */
  public Drive(
      ModuleIO FRModuleIO,
      ModuleIO FLModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      Gyro gyro) {

    // Initialize the Drive subsystem
    System.out.println("[Init] Creating Drive");
    m_gyro = gyro;
    m_modules[0] = new Module(FRModuleIO, 0);
    m_modules[1] = new Module(FLModuleIO, 1);
    m_modules[2] = new Module(BLModuleIO, 2);
    m_modules[3] = new Module(BRModuleIO, 3);

    // Initialize the swerve drive kinematics
    m_swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      m_modules[i].periodic();
    }
  }

  /**
   * Sets the entire Drive Train to either brake or coast mode
   *
   * @param enable True for brake, false for coast
   */
  public void setBrakeModeAll(boolean enable) {
    for (var module : m_modules) {
      module.setBrakeMode(enable);
    }
  }

  /**
   * Sets the Velocity of the Swerve Drive through Passing in a ChassisSpeeds (Can be Field Relative
   * OR Robot Orientated)
   *
   * @param speeds the desired chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Convert chassis speeds to Swerve Module States, these will be the setpoints for the drive and
    // turn motors
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        m_swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_S);

    // Record chassis speed and module states (setpoint)
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisStates/Setpoints", discreteSpeeds);

    // The current velocity and position of each module
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      measuredStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured states
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
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
              (m_modules[i].getPositionMeters()
                  - m_lastModulePositionsMeters[i]), // This calculates the change in angle
              m_modules[i].getAngle()); // Gets individual MODULE rotation
      m_lastModulePositionsMeters[i] = m_modules[i].getPositionMeters();
    }
    return wheelDeltas;
  }

  /**
   * Current heading of the robot. Updates based on the Gyro. If gyro is not connected, uses change
   * in module position instead
   *
   * @return The current angle of the robot
   */
  public Rotation2d getRotation() {
    Rotation2d robotYaw;

    /*
     * Twist2d is a change in distance along an arc
     * // x is the forward distance driven
     * // y is the distance driven to the side
     * // (left positive), and the component is the change in heading.
     */
    if (m_gyro.isConnected()) {
      robotYaw = m_gyro.getYaw();
    } else {
      m_twist =
          m_swerveDriveKinematics.toTwist2d(
              getWheelDeltas()); // Updates Twist Based on MODULE position
      robotYaw =
          m_lastRobotYaw.minus(
              new Rotation2d(m_twist.dtheta)); // Updates rotation 2d based on robot module position
    }
    m_lastRobotYaw = robotYaw;
    return robotYaw;
  }

  /**
   * Drives the robot with a 10% joystick deadband applied. This means joystick values between 0-0.1
   * (or 0-10%) will be ignored and not more the robot for both axises and rotation.
   *
   * <p>The joystick inputs run the robot at a percent scale from -1 (-100% reverse) to 1 (100%
   * forward)
   *
   * @param x The desired x-axis velocity from joystick
   * @param y The desired y-axis velocity from joystick
   * @param rot The desired angular velocity from joystick
   */
  public void driveWithDeadband(double x, double y, double rot) {
    // Apply deadband to x, y, and rot
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    double omega = MathUtil.applyDeadband(rot, DriveConstants.DEADBAND);

    Rotation2d linearDirection = new Rotation2d();
    if (x != 0 && y != 0) {
      linearDirection = new Rotation2d(x, y);
    }

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calculate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Run modules based on field orientated Chassis speeds
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
            this.getRotation()));
  }

  /**
   * Returns the current state of the robot
   *
   * @return the Swerve kinematics of the robot
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDriveKinematics;
  }

  /**
   * @return array of the modules positions (turn angle and drive position)
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = m_modules[i].getPosition();
    }
    return modulePositions;
  }
}
