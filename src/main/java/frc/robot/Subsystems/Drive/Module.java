package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // Closed loop PID controllers
  private final PIDController drivePID;
  private final PIDController steerPID;

  private SimpleMotorFeedforward driveFeedforward;

  public Module(ModuleIO io, int index) {
    System.out.println("[Init] Creating Module");
    this.io = io;
    this.index = index;

    drivePID =
        new PIDController(
            DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD);
    steerPID =
        new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);

    driveFeedforward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS_KRAKEN, DriveConstants.DRIVE_KV_KRAKEN);
  }

  /**
   * Put Values that Should Be Called Periodically for EACH individual Module Here. Module.periodic
   * NEEDS to be in Drive periodic OR it wont run
   */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  /** Update the inputs of the Modules */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Stops the Drive and Turn motors */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }

  /**
   * Manually sets voltage of the Drive motor
   *
   * @param volts the voltage to set the Drive motor to [-12 to 12]
   */
  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  /**
   * Manually sets voltage of the Turn motor
   *
   * @param volts the voltage to set the Turn motor to [-12 to 12]
   */
  public void setTurnVoltage(double volts) {
    io.setTurnVoltage(volts);
  }

  /**
   * Set the speed of the Drive motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. 1 representing 100
   *
   * @param percent the percent speed to set the drive motor to [-1 to 1]
   */
  public void setDrivePercentSpeed(double percent) {
    io.setDriveVoltage(percent * 12);
  }

  /**
   * Set the speed of the Turn motor based on a percent scale
   *
   * <p>On a -1 to 1 Scale. 1 representing 100
   *
   * @param percent the percent speed to set the Turn motor to [-1 to 1]
   */
  public void setTurnPercentSpeed(double percent) {
    io.setTurnVoltage(percent * 12);
  }

  /**
   * @return the current turn angle of the Module.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.turnAbsolutePositionRad);
  }

  /**
   * @return the current Drive position of the Module in meters.
   */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return the current Drive Velocity of the Module in meters per second
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
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
    io.setDriveBrakeMode(enable);
    io.setTurnBrakeMode(enable);
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
    io.setTurnVoltage(steerPID.calculate(getAngle().getRadians(), state.angle.getRadians()));

    // Update velocity based on turn error
    state.speedMetersPerSecond *= Math.cos(steerPID.getError());

    // Turn Speed m/s into Vel rad/s
    double velocityRadPerSec = state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_M;

    // Run drive controller
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + (drivePID.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)));
  }
}
