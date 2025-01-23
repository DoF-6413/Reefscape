package frc.robot.Subsystems.Drive;

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

  /** Updates logged inputs periodically */
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
}
