package frc.robot.Subsystems.AlgaeEndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface AEEIO {

  @AutoLog
  public static class AEEIOInputs {
    /** Voltage applied to the AEE motor */
    public double appliedVoltage = 0.0;
    /** Position of the AEE in radians */
    public double positionRad = 0.0;
    /** Velocity of the AEE in radians per sec */
    public double velocityRadPerSec = 0.0;
    /** Current drawn by the AEE motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the AEE motor in celsius */
    public double tempCelsius = 0.0;
  }

  /**
   * Updates the logged inputs for the AEE. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(AEEIOInputs inputs) {}

  /**
   * Sets voltage of the AEE motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}
}
