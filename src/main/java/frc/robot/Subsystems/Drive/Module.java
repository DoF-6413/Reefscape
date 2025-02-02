package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_moduleNumber;

  // Closed loop PID controllers
  private final PIDController m_drivePID;
  private final PIDController m_steerPID;

  private SimpleMotorFeedforward m_driveFeedforward;

  // private double counter = 0;
  // private final double updateFrequency = 25;

  /**
   * Constructs a new Module instance.
   *
   * <p>This creates a new Module object used to run the Drive and Turn motors of each module.
   *
   * @param io ModuleIO implementation of the current robot mode
   * @param index Module number
   */
  public Module(ModuleIO io, int moduleNumber) {
    System.out.println("[Init] Creating Module");
    m_io = io;
    m_moduleNumber = moduleNumber;

    m_drivePID =
        new PIDController(
            DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD);
    m_steerPID =
        new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);

    m_driveFeedforward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV);
    m_steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Put Values that Should Be Called Periodically for EACH individual Module Here. Module.periodic
   * NEEDS to be in Drive periodic OR it wont run
   */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drive/Module" + Integer.toString(m_moduleNumber), m_inputs);

    // if (counter % updateFrequency == 0) {
    //   this.updateRelativePosition();
    // }

    // counter++;
  }

  /**
   * Peridocially updates the logged inputs for the Module.
   *
   * @param inputs Inputs from the auto logger
   */
  public void updateInputs() {
    m_io.updateInputs(m_inputs);
  }

  /** Stops the Drive and Turn motors */
  public void stop() {
    m_io.setDriveVoltage(0.0);
    m_io.setTurnVoltage(0.0);
  }

  /**
   * Manually sets voltage of the Drive motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public void setDriveVoltage(double volts) {
    m_io.setDriveVoltage(volts);
  }

  /**
   * Manually sets voltage of the Turn motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public void setTurnVoltage(double volts) {
    m_io.setTurnVoltage(volts);
  }

  /**
   * Set the speed of the Drive motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%
   *
   * @param percent -1 (full reverse) to 1 (full forward)
   */
  public void setDrivePercentSpeed(double percent) {
    m_io.setDriveVoltage(percent * 12);
  }

  /**
   * Set the speed of the Turn motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%
   *
   * @param percent -1 (full reverse) to 1 (full forward)
   */
  public void setTurnPercentSpeed(double percent) {
    m_io.setTurnVoltage(percent * 12);
  }

  /**
   * Sets the velocity of the Drive motor using the closed loop controller built into the TalonFX
   * speed controller
   *
   * @param velocityRadPerSec Velocity to set Drive motor to in radians per second
   */
  public void setDriveVelocity(double velocityRadPerSec) {
    m_io.setDriveVelocity(velocityRadPerSec);
  }

  /**
   * Sets the position of the Turn motor using the closed loop controller built into the SparkMax
   * speed controller
   *
   * @param position Rotation2d with angle to set the Module wheel to
   */
  public void setTurnPosition(Rotation2d position) {
    m_io.setTurnPosition(position);
  }

  /**
   * The current absolute turn angle of the module in radians, normalized to a range of negative pi
   * to pi.
   *
   * @return The current turn angle of the module in radians.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(m_inputs.turnAbsolutePositionRad));
  }

  /**
   * Calculates the current drive position of the module based on the encoder readings and the wheel
   * radius.
   *
   * @return The current drive position of the module in meters.
   */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * Calculates the current linear velocity of the module based on the encoder readings and the
   * wheel radius.
   *
   * @return The current drive velocity of the module in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return the Module position (Turn angle and Drive position)
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * @return the Module state (Turn angle and Drive velocity).
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Sets the idle mode for Turn and Drive motors
   *
   * @param enable Sets break mode on true, coast on false
   */
  public void setBrakeMode(boolean enable) {
    m_io.setDriveBrakeMode(enable);
    m_io.setTurnBrakeMode(enable);
  }

  /**
   * Using a PID controller, calculates the voltage of the Drive and Turn motors based on the
   * current inputed setpoint.
   *
   * @param state Desired Swerve Module State (Desired velocity and angle)
   */
  public void runSetpoint(SwerveModuleState state) {

    // Optimize state based on current angle, aka take the shortest path for wheel to reach desired
    // angle in rad (-pi,pi))
    state.optimize(getAngle());

    // Run turn controller
    m_io.setTurnVoltage(m_steerPID.calculate(getAngle().getRadians(), state.angle.getRadians()));

    // Update velocity based on turn error
    // state.speedMetersPerSecond *= Math.cos(m_steerPID.getError());

    // Turn Speed m/s into Vel rad/s
    double velocityRadPerSec = state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_M;

    // Run drive controller
    // m_io.setDriveVoltage(
    //     m_driveFeedforward.calculate(velocityRadPerSec)
    //         + m_drivePID.calculate(m_inputs.driveVelocityRadPerSec, velocityRadPerSec));
    m_io.setDriveVelocity(velocityRadPerSec);
  }

  /**
   * Sets the PID values for the Drive motor's built in closed loop controller
   *
   * @param kP P gain value
   * @param kI I gain value
   * @param kD D gain value
   */
  public void setDrivePID(double kP, double kI, double kD) {
    m_io.setDrivePID(kP, kI, kD);
  }

  /**
   * Sets the FF values for the Drive motor's built in closed loop controller
   *
   * @param kS S gain value
   * @param kV V gain value
   */
  public void setDriveFF(double kS, double kV) {
    m_io.setDriveFF(kS, kV);
  }

  /**
   * Sets the PID values for the Turn motor's built in closed loop controller
   *
   * @param kP P gain value
   * @param kI I gain value
   * @param kD D gain value
   */
  public void setTurnPID(double kP, double kI, double kD) {
    m_steerPID.setPID(kP, kI, kD);
  }

  // public void updateRelativePosition() {
  //   m_io.updateRelativePosition();
  // }
}
