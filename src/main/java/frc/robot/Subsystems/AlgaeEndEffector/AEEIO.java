package frc.robot.Subsystems.AlgaeEndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface AEEIO {

  @AutoLog
  public static class AEEIOInputs {
    /** Voltage that AEE motor draws */
    public double appliedVoltage = 0.0;
    /** Position of the wheel in radians */
    public double positionRad = 0.0;
    /** Velocity of the wheel in radians per sec */
    public double velocityRadPerSec = 0.0;
    /** Current drawn by the motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the motor in celsius */
    public double tempCelsius = 0.0;

    public boolean isConnected = false;
  }

  /**
   * Peridocially updates the logged inputs for the Module.
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(AEEIOInputs inputs) {}

  /**
   * Manually sets voltage of the AEE motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public default void setVoltage(double volts) {}
}
