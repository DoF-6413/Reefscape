package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

/** IO Interface to log the inputs of and create the default methods for the Swerve Modules */
public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    // Drive motor
    /** Voltage applied to the Drive motor */
    public double driveAppliedVoltage = 0.0;
    /** Distance driven by the Module wheel in radians */
    public double drivePositionRad = 0.0;
    /** Velocity of the Module wheel driven by the Drive motor in radians per sec */
    public double driveVelocityRadPerSec = 0.0;
    /** Current drawn by the Drive motor in amps */
    public double driveCurrentAmps = 0.0;
    /** Temperature of the Drive motor in celsius */
    public double driveTempCelsius = 0.0;
    /** Whether a singal is being recieved by the Drive motor or not */
    public boolean driveIsConnected = false;

    // Turn motor
    /** Voltage applied to the Turn motor */
    public double turnAppliedVoltage = 0.0;
    /** Absolute position of the wheel angle in radians (CANcoder) */
    public double turnAbsolutePositionRad = 0.0;
    /** Velocity of the Module wheel driven by the Turn motor in radians per sec (CANcoder) */
    public double turnVelocityRadPerSec = 0.0;
    /** Current drawn by the Turn motor in amps */
    public double turnCurrentAmps = 0.0;
    /** Temperature of the Turn motor in celsius */
    public double turnTempCelsius = 0.0;
    /** Whether a singal is being recieved by the CANcoder or not */
    public boolean absoluteEncoderIsConnected = false;
  }

  /**
   * Updates the logged inputs for the Module. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Sets voltage of the Drive motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setDriveVoltage(double volts) {}

  /**
   * Sets voltage of the Turn motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setTurnVoltage(double volts) {}

  /**
   * Sets the idle mode for the Drive motor
   *
   * @param enable Sets brake mode on true, coast on false
   */
  public default void setDriveBrakeMode(boolean enable) {}

  /**
   * Sets the idle mode for the Turn motor
   *
   * @param enable Sets brake mode on true, coast on false
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
   * Sets the PID values for the Drive motor's built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /**
   * Sets the Feedforward values for the Drive motor's built in closed loop controller
   *
   * @param kS Static gain value
   * @param kV Velocity gain value
   */
  public default void setDriveFF(double kS, double kV) {}
}
