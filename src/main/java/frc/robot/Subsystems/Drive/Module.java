package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  public Module(ModuleIO io, int index) {
    System.out.println("[Init] Creating Module");
    this.io = io;
    this.index = index;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }
  /** Stops the Robot */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }


  /** Manually Sets Voltage of the Drive Motor in Individual Module (Max is 12 Volts) */
  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  /** Manually Sets Voltage of the Turn Motor in Individual Module (Max is 12 Volts) */
  public void setTurnVoltage(double volts) {
    io.setTurnVoltage(volts);
  }

  /**
   * Manually Sets the Percent Speed of the Drive Motor in Individual Module (On a -1 to 1 Scale. 1
   * representing 100)
   */
  public void setDrivePercentSpeed(double percent) {
    io.setDriveVoltage(percent * 12);
  }

  /**
   * Manually Sets the Percent Speed of the Turn Motor in Individual Module (On a -1 to 1 Scale. 1
   * representing 100)
   */
  public void setTurnPercentSpeed(double percent) {
    io.setTurnVoltage(percent * 12);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    // Angle Modulus sets the Value Returned to be on a -pi, pi scale
    return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return the current drive velocity of the module in meters per second
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
  }

  /**
   * @return the module position (turn angle and drive position)
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * @return the module state (turn angle and drive velocity).
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Sets Break Mode for Turn and Drive Motors
   *
   * @param enable enables break mode on true, coast on false
   */
  public void setBrakeModeAll(boolean enable) {
    io.setDriveBrakeMode(enable);
    io.setTurnBrakeMode(enable);
  }

  /**
   * Put Values that Should Be Called Periodically for EACH individual Module Here. Module.periodic
   * NEEDS to be in Drive periodic OR it wont run
   */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }
}

