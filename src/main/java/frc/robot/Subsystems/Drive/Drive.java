// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Gyro.Gyro;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // Chassis
  private final Module[] m_modules = new Module[4];
  private final Gyro m_gyro;
  public final SwerveDriveKinematics m_swerveDriveKinematics;

  // Robot rotation
  private Twist2d m_twist = new Twist2d();
  private double[] m_lastModulePositionsMeters = new double[4];
  public Rotation2d m_lastRobotYaw = new Rotation2d();

  // System ID
  private final SysIdRoutine m_sysId;

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
    System.out.println("[Init] Creating Drive");

    m_gyro = gyro;
    m_modules[0] = new Module(FRModuleIO, 0); // Index 0 corresponds to front right Module
    m_modules[1] = new Module(FLModuleIO, 1); // Index 1 corresponds to front left Module
    m_modules[2] = new Module(BLModuleIO, 2); // Index 2 corresponds to back left Module
    m_modules[3] = new Module(BRModuleIO, 3); // Index 3 corresponds to back right Module

    m_swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());

    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "/SysId/Drive/SysId State",
                        state
                            .toString())), // Log SysId to AdvantageScope rather than the WPI Logger
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Tunable PID & Feedforward values
    SmartDashboard.putBoolean("PIDFF/Drive/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/Drive/Drive_kP", DriveConstants.DRIVE_KP);
    SmartDashboard.putNumber("PIDFF/Drive/Drive_kI", DriveConstants.DRIVE_KI);
    SmartDashboard.putNumber("PIDFF/Drive/Drive_kD", DriveConstants.DRIVE_KD);
    SmartDashboard.putNumber("PIDFF/Drive/Drive_kS", DriveConstants.DRIVE_KS);
    SmartDashboard.putNumber("PIDFF/Drive/Drive_kV", DriveConstants.DRIVE_KV);
    SmartDashboard.putNumber("PIDFF/Drive/Turn_kP", DriveConstants.TURN_KP);
    SmartDashboard.putNumber("PIDFF/Drive/Turn_kI", DriveConstants.TURN_KI);
    SmartDashboard.putNumber("PIDFF/Drive/Turn_kD", DriveConstants.TURN_KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update the periodic for each Module
    for (int i = 0; i < 4; i++) {
      m_modules[i].periodic();
    }

    // Enable and update tunable PID values through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF/Drive/EnableTuning", false)) {
      this.updateDrivePID();
      this.updateDriveFF();
      this.updateTurnPID();
    }
  }

  /**
   * Sets the entire Drivetrain to either brake or coast mode
   *
   * @param enable True for brake, false for coast
   */
  public void setBrakeModeAll(boolean enable) {
    for (var module : m_modules) {
      module.setBrakeMode(enable);
    }
  }

  /**
   * Sets the velocity of the Swerve Drive through passing in a ChassisSpeeds (can be Field Relative
   * OR Robot Orientated) that contains the desired linear and angular velocities for the robot
   *
   * @param speeds The desired ChassisSpeeds
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
    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      measuredStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured States
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
  }

  /**
   * Run each Module at a specified linear speed and angle.
   *
   * @param setpointStates An array of SwerveModuleStates (Module speed in m/s, and the Module angle
   *     in radians).
   */
  public void runSwerveModules(SwerveModuleState[] setpointStates) {
    // Record initial Module States setpoint
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];

    // Run each Module
    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      optimizedStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured States
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", optimizedStates);
  }

  /**
   * @return Swerve kinematics configuration of the robot
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDriveKinematics;
  }

  /**
   * @return The position of each Module (distance travelled and wheel angles)
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    // Retrieve SwerveModulePosition for each Module
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = m_modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Runs the Drivetrain with inputed velocities
   *
   * @param x Linear velociy (m/s) in x direction of Entire Swerve Drive
   * @param y Linear velocity (m/s) in y direction of Entire Swerve Drive
   * @param rot Angular velocity (rad/s) of Entire Swerve Drive
   */
  public void setRaw(double xVelcoity, double yVelcoity, double angularVelocity) {
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelcoity, yVelcoity, angularVelocity, this.getRotation()));
  }

  /**
   * @return A list of SwerveModulePositions containing the change in Module position and angle
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
   * Current heading of the robot. Updates based on the Gyro. If the Gyro isn't connected, uses
   * change in Module Position instead
   *
   * @return The current angle of the robot
   */
  public Rotation2d getRotation() {
    Rotation2d robotYaw;

    /*
     * Twist2d is a change in distance along an arc
     * x is the forward distance driven
     * y is the distance driven to the side (left positive),
     * and the component is the change in heading.
     */
    if (m_gyro.isConnected()) {
      // Updates heading based on Gyro reading
      robotYaw = m_gyro.getYaw();
    } else {
      // Updates heading based on change in Module Position
      m_twist = m_swerveDriveKinematics.toTwist2d(getWheelDeltas());
      robotYaw = m_lastRobotYaw.minus(new Rotation2d(m_twist.dtheta));
    }
    // Save heading for next call
    m_lastRobotYaw = robotYaw;
    return robotYaw;
  }

  /**
   * Locks Module orientation at 0 degrees and runs Drive motors at specified voltage
   *
   * @param output Voltage
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  /**
   * @param direction Forward or Reverse direction
   * @return A quasistatic test in the specified direction
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0)) // Allows Module positions to reset
        .withTimeout(1.0)
        .andThen(m_sysId.quasistatic(direction));
  }

  /**
   * @param direction Forward or Reverse direction
   * @return A dynamic test in the specified direction
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0)) // Allows Module positions to reset
        .withTimeout(1.0)
        .andThen(m_sysId.dynamic(direction));
  }

  /**
   * @return Average velocity of Drive motors in rotations per second, for FeedForward
   *     characterization
   */
  public double getAverageDriveVelocity() {
    double velocity = 0.0;
    for (int i = 0; i < 4; i++) {
      velocity += Units.radiansToRotations(m_modules[i].getVelocityRadPerSec());
    }
    return velocity;
  }

  /**
   * @return Current linear and angular speed of the robot based on the current State of each Module
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
   * Sets the PID values for all Drive motors' built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setDrivePID(double kP, double kI, double kD) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDrivePID(kP, kI, kD);
    }
  }

  /**
   * Sets the Feedforward values for all Drive motors' built in closed loop controller
   *
   * @param kS Static gain value
   * @param kV Velocity gain value
   */
  public void setDriveFF(double kS, double kV) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDriveFF(kS, kV);
    }
  }

  /**
   * Sets the PID values for all Turn motors' in-code PID controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setTurnPID(double kP, double kI, double kD) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setTurnPID(kP, kI, kD);
    }
  }

  /** Update PID values for the Drive motors from SmartDashboard inputs */
  private void updateDrivePID() {
    if (DriveConstants.DRIVE_KP
            != SmartDashboard.getNumber("PIDFF/Drive/Drive_kP", DriveConstants.DRIVE_KP)
        || DriveConstants.DRIVE_KI
            != SmartDashboard.getNumber("PIDFF/Drive/Drive_kI", DriveConstants.DRIVE_KI)
        || DriveConstants.DRIVE_KD
            != SmartDashboard.getNumber("PIDFF/Drive/Drive_kD", DriveConstants.DRIVE_KD)) {
      DriveConstants.DRIVE_KP =
          SmartDashboard.getNumber("PIDFF/Drive/Drive_kP", DriveConstants.DRIVE_KP);
      DriveConstants.DRIVE_KI =
          SmartDashboard.getNumber("PIDFF/Drive/Drive_kI", DriveConstants.DRIVE_KI);
      DriveConstants.DRIVE_KD =
          SmartDashboard.getNumber("PIDFF/Drive/Drive_kD", DriveConstants.DRIVE_KD);
      this.setDrivePID(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD);
    }
  }

  /** Update FeedForward values for the Drive motors from SmartDashboard inputs */
  private void updateDriveFF() {
    if (DriveConstants.DRIVE_KS
            != SmartDashboard.getNumber("PIDFF/Drive/Drive_kS", DriveConstants.DRIVE_KS)
        || DriveConstants.DRIVE_KV
            != SmartDashboard.getNumber("PIDFF/Drive/Drive_kV", DriveConstants.DRIVE_KV)) {
      DriveConstants.DRIVE_KS =
          SmartDashboard.getNumber("PIDFF/Drive/Drive_kS", DriveConstants.DRIVE_KS);
      DriveConstants.DRIVE_KV =
          SmartDashboard.getNumber("PIDFF/Drive/Drive_kV", DriveConstants.DRIVE_KV);
      this.setDriveFF(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV);
    }
  }

  /** Update PID values for the Turn motors from SmartDashboard inputs */
  private void updateTurnPID() {
    if (DriveConstants.TURN_KP
            != SmartDashboard.getNumber("PIDFF/Drive/Turn_kP", DriveConstants.TURN_KP)
        || DriveConstants.TURN_KI
            != SmartDashboard.getNumber("PIDFF/Drive/Turn_kI", DriveConstants.TURN_KI)
        || DriveConstants.TURN_KD
            != SmartDashboard.getNumber("PIDFF/Drive/Turn_kD", DriveConstants.TURN_KD)) {
      DriveConstants.TURN_KP =
          SmartDashboard.getNumber("PIDFF/Drive/Turn_kP", DriveConstants.TURN_KP);
      DriveConstants.TURN_KI =
          SmartDashboard.getNumber("PIDFF/Drive/Turn_kI", DriveConstants.TURN_KI);
      DriveConstants.TURN_KD =
          SmartDashboard.getNumber("PIDFF/Drive/Turn_kD", DriveConstants.TURN_KD);
      Logger.recordOutput("/Drive/PIDFF/Turn_kP", DriveConstants.TURN_KP);
      Logger.recordOutput("/Drive/PIDFF/Turn_kI", DriveConstants.TURN_KI);
      Logger.recordOutput("/Drive/PIDFF/Turn_kD", DriveConstants.TURN_KD);
      this.setTurnPID(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
    }
  }
}
