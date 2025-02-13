package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    /** Voltage that drive motor draws */
    public double driveAppliedVoltage = 0.0;
    /** Position of the wheel in radians */
    public double drivePositionRad = 0.0;
    /** Velocity of the wheel in radians per sec */
    public double driveVelocityRadPerSec = 0.0;
    /** Current drawn by the motor in amps */
    public double driveCurrentAmps = 0.0;
    /** Temperature of the motor in celsius */
    public double driveTempCelsius = 0.0;
    /** If a singal is being recieved from the Drive motor */
    public boolean driveIsConnected = false;

    /** Voltage that turn motor draws */
    public double turnAppliedVoltage = 0.0;
    /** Relative position of the wheel in radians (NEO encoder) */
    public double turnPositionRad = 0.0;
    /** Absolute position of the wheel in radians (CANcoder) */
    public double turnAbsolutePositionRad = 0.0;
    /** Turn velocity of the wheel in radians per sec (CANcoder) */
    public double turnVelocityRadPerSec = 0.0;
    /** Current drawn by the motor in amps */
    public double turnCurrentAmps = 0.0;
    /** Temperature of the motor in celsius */
    public double turnTempCelsius = 0.0;
    /** If a singal is being recieved from the CANcoder */
    public boolean absoluteEncoderIsConnected = false;
  }

  /**
   * Peridocially updates the logged inputs for the Module.
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Manually sets voltage of the Drive motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public default void setDriveVoltage(double volts) {}

  /**
   * Manually sets voltage of the Turn motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public default void setTurnVoltage(double volts) {}

  /**
   * Sets the idle mode for the Drive motor
   *
   * @param enable Sets break mode on true, coast on false
   */
  public default void setDriveBrakeMode(boolean enable) {}

  /**
   * Sets the idle mode for the Turn motor
   *
   * @param enable Sets break mode on true, coast on false
   */
  public default void setTurnBrakeMode(boolean enable) {}

  /**
   * Sets the velocity of the Drive motor using the closed loop controller built into the TalonFX
   * speed controller
   *
   * @param velocityRadPerSec Velocity to set Drive motor to in radians per second
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /**
   * Sets the position of the Turn motor using the closed loop controller built into the SparkMax
   * speed controller
   *
   * @param position Rotation2d with angle to set the Module wheel to
   */
  public default void setTurnPosition(Rotation2d position) {}

  /**
   * Sets the PID values for the Drive motor's built in closed loop controller
   *
   * @param kP P gain value
   * @param kI I gain value
   * @param kD D gain value
   */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /**
   * Sets the FF values for the Drive motor's built in closed loop controller
   *
   * @param kS S gain value
   * @param kV V gain value
   */
  public default void setDriveFF(double kS, double kV) {}

  /**
   * Sets the PID values for the Turn motor's built in closed loop controller
   *
   * @param kP P gain value
   * @param kI I gain value
   * @param kD D gain value
   */
  // public default void setTurnPID(double kP, double kI, double kD) {}

  // public default void updateRelativePosition() {}
}
