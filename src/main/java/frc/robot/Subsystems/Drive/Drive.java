// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Utils.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  /* Chassis */
  // Modules
  private final Module[] m_modules = new Module[4];
  private final SwerveDriveKinematics m_swerveDriveKinematics;
  // Gyro
  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  // Robot rotation
  public Rotation2d m_robotHeading = new Rotation2d();
  private Twist2d m_twist = new Twist2d();
  private SwerveModulePosition[] m_lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Swerve Pose Estimator Objects
  private final SwerveDrivePoseEstimator m_swervePoseEstimator;

  // Odometry reading lock
  static final Lock odometryLock = new ReentrantLock();

  /**
   * Constructs a new {@link Drive} instance.
   *
   * <p>This creates a new Drive {@link SubsystemBase} object which determines whether the methods
   * and inputs are initialized with the real, sim, or replay code. The Drivetrain consists of four
   * {@link Module} and an IMU (Gyro) sensor.
   *
   * @param FLModuleIO Front Left {@link ModuleIO} implementation of the current robot mode.
   * @param FRModuleIO Front Right {@link ModuleIO} implementation of the current robot mode.
   * @param BLModuleIO Back Left {@link ModuleIO} implementation of the current robot mode.
   * @param BRModuleIO Back Right {@link ModuleIO} implementation of the current robot mode.
   * @param gyroIO {@link GyroIO} implementation of the current robot mode.
   */
  public Drive(
      ModuleIO FLModuleIO,
      ModuleIO FRModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      GyroIO gyroIO) {
    System.out.println("[Init] Creating Drive");

    // Initialize Drivetrain and Gyro
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(FLModuleIO, 0); // Index 0 corresponds to front right Module
    m_modules[1] = new Module(FRModuleIO, 1); // Index 1 corresponds to front left Module
    m_modules[2] = new Module(BLModuleIO, 2); // Index 2 corresponds to back left Module
    m_modules[3] = new Module(BRModuleIO, 3); // Index 3 corresponds to back right Module

    // Initialize utilities
    m_swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());
    PhoenixOdometryThread.getInstance().start();

    // Configure PathPlanner
    AutoBuilder.configure(
        this::getCurrentPose2d,
        this::resetPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(
                PathPlannerConstants.TRANSLATION_KP, 0, PathPlannerConstants.TRANSLATION_KD),
            new PIDConstants(
                PathPlannerConstants.ROTATION_KP, 0, PathPlannerConstants.ROTATION_KD)),
        PathPlannerConstants.ROBOT_CONFIG,
        // Mirror the paths to the red side of the field if true
        () ->
            DriverStation.getAlliance().isPresent()
                && RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red,
        this);
    // Pathfinder by FRC 6328 that adds AdvantageKit logging functionality to PathPlanner's
    // Pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Initialize Pose Estimator
    m_swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            m_swerveDriveKinematics, m_robotHeading, this.getModulePositions(), new Pose2d());

    // Tunable PID & Feedforward gains
    SmartDashboard.putBoolean("PIDFF_Tuning/Drive/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Drive_kP", DriveConstants.DRIVE_KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Drive_kI", DriveConstants.DRIVE_KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Drive_kD", DriveConstants.DRIVE_KD);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Drive_kS", DriveConstants.DRIVE_KS);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Drive_kV", DriveConstants.DRIVE_KV);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Turn_kP", DriveConstants.TURN_KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Turn_kI", DriveConstants.TURN_KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Drive/Turn_kD", DriveConstants.TURN_KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Prevents odometry updates while reading data
    odometryLock.lock();
    // Update the periodic for each Module and the Gyro
    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.processInputs("Gyro", m_gyroInputs);
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs();
    }
    // Re-enable odometry updates
    odometryLock.unlock();

    // Run the periodic of each Module
    for (var module : m_modules) {
      module.periodic();
    }

    // Stop the motors when disabled so robot doesn't jerk when re-enabled
    if (DriverStation.isDisabled()) {
      this.runVelocity(new ChassisSpeeds());
    }

    // Update odometry with queued readings from motors and encoders in the form of
    // SwerveModulePositions
    double[] sampleTimestamps = m_modules[0].getOdometryTimestamps();
    for (int i = 0; i < sampleTimestamps.length; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];

      // Get wheel positions and deltas from each Module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelPositions[moduleIndex] = m_modules[moduleIndex].getOdometryPositions()[i];
        wheelDeltas[moduleIndex] =
            new SwerveModulePosition(
                wheelPositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                wheelPositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = wheelPositions[moduleIndex];
      }

      // Update robot heading
      if (m_gyroInputs.connected) {
        // Use Pigeon IMU
        m_robotHeading = m_gyroInputs.odometryYawPositions[i];
      } else {
        // Calculate heading change based on change in Module Position as a fallback
        m_twist = m_swerveDriveKinematics.toTwist2d(wheelDeltas);
        m_robotHeading = m_robotHeading.plus(Rotation2d.fromRadians(m_twist.dtheta));
      }

      // Apply odometry update
      m_swervePoseEstimator.updateWithTime(sampleTimestamps[i], m_robotHeading, wheelPositions);

      Logger.recordOutput("Odometry/EstimatedPose", m_swervePoseEstimator.getEstimatedPosition());
    }

    // Enable and update tunable PID and Feedforward gains through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/Drive/EnableTuning", false)) {
      this.updateDrivePID();
      this.updateDriveFF();
      this.updateTurnPID();
    }
  }

  /**
   * Sets the idle mode of the entire Drivetrain (Drive and Turn motors).
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeModeAll(boolean enable) {
    for (var module : m_modules) {
      module.enableBrakeMode(enable);
    }
  }

  /* ~~~~~~~~~~~~~~~~~~ Chassis and Modules ~~~~~~~~~~~~~~~~~~ */

  /**
   * Sets the velocity of the Swerve Drive through passing in a {@link ChassisSpeeds} (can be Field
   * Relative OR Robot Orientated) that contains the desired linear and angular velocities for the
   * robot.
   *
   * @param speeds The desired {@link ChassisSpeeds}.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to SwerveModuleStates, these will be the setpoints for the Drive and
    // Turn motors
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        m_swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_S);

    // Record ChassisSpeeds and initial Module States setpoint
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisStates/Setpoints", discreteSpeeds);

    // The current velocity and position of each Module
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    // Run the Modules and retrieve their State (velocity and angle)
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      measuredStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured Module States
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
  }

  /**
   * Run each Module at a specified linear speed and angle.
   *
   * @param setpointStates An array of {@link SwerveModuleState} (Module speed in m/s, and the
   *     Module angle in radians).
   */
  public void runSwerveModules(SwerveModuleState[] setpointStates) {
    // Record initial Module States setpoint
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];

    // Run each Module
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      optimizedStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured States
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", optimizedStates);
  }

  /**
   * @return {@link SwerveDriveKinematics} configuration of the robot.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDriveKinematics;
  }

  /**
   * @return An array of {@link SwerveModulePosition} for each Module (distance travelled and wheel
   *     angles).
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    // Retrieve SwerveModulePosition for each Module
    for (int i = 0; i < m_modules.length; i++) {
      modulePositions[i] = m_modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Runs the Drivetrain with inputed velocities in field relative mode.
   *
   * @param xVelocity Linear velocity (m/s) in x direction of the entire Swerve Drive.
   * @param yVelocity Linear velocity (m/s) in y direction of the entire Swerve Drive.
   * @param angularVelocity Angular velocity (rad/s) of the entire Swerve Drive.
   */
  public void setRaw(double xVelocity, double yVelocity, double angularVelocity) {
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, angularVelocity, this.getRobotHeading()));
  }

  /**
   * @return Current linear and angular speed of the robot based on the current State of each
   *     Module.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_swerveDriveKinematics.toChassisSpeeds(
        new SwerveModuleState[] {
          m_modules[0].getState(),
          m_modules[1].getState(),
          m_modules[2].getState(),
          m_modules[3].getState(),
        });
  }

  /**
   * Current heading of the robot. Updates based on the Gyro. If the Gyro isn't connected, uses
   * change in Module Position instead. Value returned from the {@link SwerveDrivePoseEstimator}
   * which may get affected by Vision measurements.
   *
   * @return {@link Rotation2d} of the current angle of the robot.
   */
  public Rotation2d getRobotHeading() {
    return m_robotHeading;
  }

  /* ~~~~~~~~~~~~~~~~~~ Gyro ~~~~~~~~~~~~~~~~~~ */
  /**
   * The angle of the Gyro is normalized to a range of negative pi to pi.
   *
   * @return {@link Rotation2d} of yaw angle (about the Z Axis) of the robot in radians.
   */
  public Rotation2d getGyroAngle() {
    return m_gyroInputs.yawPositionRad;
  }

  /**
   * @return Angular velocity (about the z-axis) of the robot in radians per second.
   */
  public double getYawAngularVelocity() {
    return m_gyroInputs.yawVelocityRadPerSec;
  }

  /** Resets the robot heading to the front side of the robot, making it the new 0 degree angle. */
  public void zeroYaw() {
    m_gyroIO.zeroHeading();
  }

  /* ~~~~~~~~~~~~~~~~~~ Pose Estimator ~~~~~~~~~~~~~~~~~~ */

  /**
   * @return {@link Pose2d} (x, y, rotation) of the robot on the field.
   */
  public Pose2d getCurrentPose2d() {
    return m_swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current position of the robot.
   *
   * @param pose {@link Pose2d} to set the robot to.
   */
  public void resetPose(Pose2d pose) {
    m_swervePoseEstimator.resetPosition(m_robotHeading, this.getModulePositions(), pose);
  }

  /**
   * Adds Vision measurements to the {@link SwerveDrivePoseEstimator}.
   *
   * @param visionPoseEstimation {@link Pose2d} calculated from AprilTag.
   * @param timestampSec Timestamp when position was calculated in seconds.
   * @param visionStdDevs Standard deviation from the average calculation (distance & angle).
   */
  public void addVisionMeasurement(
      Pose2d visionPoseEstimation, double timestampSec, Matrix<N3, N1> visionStdDevs) {
    m_swervePoseEstimator.addVisionMeasurement(visionPoseEstimation, timestampSec, visionStdDevs);
  }

  /* ~~~~~~~~~~~~~~~~~~ Wheel Radius Characterization ~~~~~~~~~~~~~~~~~~ */

  /**
   * Locks Module orientation at 0 degrees and runs Drive motors at specified voltage.
   *
   * @param output Voltage
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  /**
   * @return Average velocity of Drive motors in rotations per second, for Feedforward
   *     characterization.
   */
  public double getAverageDriveVelocity() {
    double velocity = 0.0;
    for (int i = 0; i < m_modules.length; i++) {
      velocity += Units.radiansToRotations(m_modules[i].getVelocityRadPerSec());
    }
    return velocity;
  }

  /**
   * @return A double array containing the positions of the Drive motors in radians.
   */
  public double[] getDrivePositionRad() {
    double[] positions = new double[4];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPositionRad();
    }
    return positions;
  }

  /* ~~~~~~~~~~~~~~~~~~ PID and Feedforward ~~~~~~~~~~~~~~~~~~ */

  /**
   * Sets the PID gains for all Drive motors' built in closed loop controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setDrivePID(double kP, double kI, double kD) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDrivePID(kP, kI, kD);
    }
  }

  /**
   * Sets the Feedforward gains for all Drive motors' built in closed loop controller.
   *
   * @param kS Static gain value.
   * @param kV Velocity gain value.
   */
  public void setDriveFF(double kS, double kV) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDriveFF(kS, kV);
    }
  }

  /**
   * Sets the PID gains for all Turn motors' in-code PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setTurnPID(double kP, double kI, double kD) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setTurnPID(kP, kI, kD);
    }
  }

  /** Update PID gains for the Drive motors from SmartDashboard inputs. */
  private void updateDrivePID() {
    // If any value on SmartDashboard changes, update the gains
    if (DriveConstants.DRIVE_KP
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kP", DriveConstants.DRIVE_KP)
        || DriveConstants.DRIVE_KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kI", DriveConstants.DRIVE_KI)
        || DriveConstants.DRIVE_KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kD", DriveConstants.DRIVE_KD)) {
      DriveConstants.DRIVE_KP =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kP", DriveConstants.DRIVE_KP);
      DriveConstants.DRIVE_KI =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kI", DriveConstants.DRIVE_KI);
      DriveConstants.DRIVE_KD =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kD", DriveConstants.DRIVE_KD);
      // Sets the new gains
      this.setDrivePID(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD);
    }
  }

  /** Update Feedforward gains for the Drive motors from SmartDashboard inputs. */
  private void updateDriveFF() {
    // If any value on SmartDashboard changes, update the gains
    if (DriveConstants.DRIVE_KS
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kS", DriveConstants.DRIVE_KS)
        || DriveConstants.DRIVE_KV
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kV", DriveConstants.DRIVE_KV)) {
      DriveConstants.DRIVE_KS =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kS", DriveConstants.DRIVE_KS);
      DriveConstants.DRIVE_KV =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Drive_kV", DriveConstants.DRIVE_KV);
      // Sets the new gains
      this.setDriveFF(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV);
    }
  }

  /** Update PID gains for the Turn motors from SmartDashboard inputs. */
  private void updateTurnPID() {
    // If any value on SmartDashboard changes, update the gains
    if (DriveConstants.TURN_KP
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kP", DriveConstants.TURN_KP)
        || DriveConstants.TURN_KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kI", DriveConstants.TURN_KI)
        || DriveConstants.TURN_KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kD", DriveConstants.TURN_KD)) {
      DriveConstants.TURN_KP =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kP", DriveConstants.TURN_KP);
      DriveConstants.TURN_KI =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kI", DriveConstants.TURN_KI);
      DriveConstants.TURN_KD =
          SmartDashboard.getNumber("PIDFF_Tuning/Drive/Turn_kD", DriveConstants.TURN_KD);
      // Sets the new gains
      this.setTurnPID(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
    }
  }
}
