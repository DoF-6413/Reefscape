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
    /** Relative position of the wheel in radians */
    public double turnPositionRad = 0.0;
    /** Absolute position of the wheel in radians */
    public double turnAbsolutePositionRad = 0.0;
    /** Turn velocity of the wheel in radians per sec */
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

  /** Overrides the drive voltage */
  public default void setDriveVoltage(double voltage) {}

  /** Overrides the turn voltage */
  public default void setTurnVoltage(double voltage) {}

  /** Enables brake mode for drive */
  public default void setDriveBrakeMode(boolean brake) {}

  /** Enables brake mode for turn */
  public default void setTurnBrakeMode(boolean brake) {}
}
