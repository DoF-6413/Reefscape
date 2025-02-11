package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_moduleNumber;

  // PID controllers
  private final PIDController m_steerPID;

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

    m_steerPID =
        new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);

    // Considers min and max the same point, required for the Swerve Modules since we wrap the
    // position from -pi to pi
    m_steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update and log IO inputs from logger
   * 
   * <p>Put values that should be called periodically for EACH individual Module here. Module.periodic()
   * NEEDS to be called in Drive.periodic() OR ELSE it wont run
   */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drive/Module" + Integer.toString(m_moduleNumber), m_inputs);
  }

  /**
   * Updates the logged inputs for the Module. Must be called periodically
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
   * Sets voltage of the Drive motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setDriveVoltage(double volts) {
    m_io.setDriveVoltage(volts);
  }

  /**
   * Sets voltage of the Turn motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setTurnVoltage(double volts) {
    m_io.setTurnVoltage(volts);
  }

  /**
   * Sets the speed of the Drive motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%
   *
   * @param percent -1 (full reverse speed) to 1 (full forward speed)
   */
  public void setDrivePercentSpeed(double percent) {
    m_io.setDriveVoltage(percent * 12);
  }

  /**
   * Set the speed of the Turn motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. -1 representing -100%, 1 representing 100%
   *
   * @param percent -1 (full reverse speed) to 1 (full forward speed)
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
   * The current absolute Turn angle of the Module in radians, normalized to a range of negative pi
   * to pi.
   *
   * @return The current Turn angle of the Module in radians.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(m_inputs.turnAbsolutePositionRad));
  }

  /**
   * Calculates the Drive linear displacement of the Module based on the encoder readings (angular position) and the wheel
   * radius.
   *
   * @return The current Drive position of the Module in meters.
   */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * Calculates the current linear velocity of the Module based on the encoder readings (angular velocity) and the
   * wheel radius.
   *
   * @return The current Drive velocity of the Module in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return The current velocity of the Drive motor in rad per sec
   */
  public double getVelocityRadPerSec() {
    return m_inputs.driveVelocityRadPerSec;
  }

  /**
   * @return the current SwerveModulePosition (Turn angle and Drive position)
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * @return the current SwerveModuleState (Turn angle and Drive velocity).
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
   * @param state Desired SwerveModuleState (Desired linear speed and wheel angle)
   */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle, aka take the shortest path for wheel to reach desired
    // angle in rad (-pi,pi))
    state.optimize(getAngle());

    // Run Turn motor through a PID loop
    m_io.setTurnVoltage(m_steerPID.calculate(getAngle().getRadians(), state.angle.getRadians()));

    // Update velocity based on Turn error
    // state.speedMetersPerSecond *= Math.cos(m_steerPID.getError()); // TODO: test and verify is needed

    // Turn Speed m/s into Vel rad/s
    double velocityRadPerSec = state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_M;

    // Runs the Drive motor through the TalonFX closed loop controller
    m_io.setDriveVelocity(velocityRadPerSec);
  }

  /**
   * Sets the PID values for the Drive motor's built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setDrivePID(double kP, double kI, double kD) {
    m_io.setDrivePID(kP, kI, kD);
  }

  /**
   * Sets the FF values for the Drive motor's built in closed loop controller
   *
   * @param kS Static gain value
   * @param kV Velocity gain value
   */
  public void setDriveFF(double kS, double kV) {
    m_io.setDriveFF(kS, kV);
  }

  /**
   * Sets the PID values for the Turn motor's built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setTurnPID(double kP, double kI, double kD) {
    m_steerPID.setPID(kP, kI, kD);
  }

  /**
   * Locks Module orientation at 0 degrees and runs Drive motor at specified voltage
   *
   * @param output Voltage
   */
  public void runCharacterization(double output) {
    m_io.setDriveVoltage(output);
    m_io.setTurnVoltage(m_steerPID.calculate(getAngle().getRadians(), 0)); // Setpoint at 0 degrees
  }
}
