package frc.robot.Subsystems.CoralEndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface CEEIO {

  @AutoLog
  public static class CEEIOInputs {
    /** Voltage applied to the CEE motor */
    public double appliedVoltage = 0.0;
    /** Velocity of the CEE in radians per sec */
    public double velocityRadPerSec = 0.0;
    /** Current drawn by the CEE motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the CEE motor in celsius */
    public double tempCelsius = 0.0;
  }

  /**
   * Updates the logged inputs for the CEE. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(CEEIOInputs inputs) {}

  /**
   * Sets voltage of the CEE motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}
}
