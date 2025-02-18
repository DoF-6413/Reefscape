package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {

  @AutoLog
  public static class FunnelIOInputs {
    /** Voltage applied to the AEE motor */
    public double appliedVoltage = 0.0;
    /** Velocity of the Funnel in radians per second */
    public double velocityRadPerSec = 0.0;
    /** Current drawn by the Funnel motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the Funnel motor in celsius */
    public double tempCelsius = 0.0;
  }

  /**
   * Updates the logged inputs for the Funnel. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(FunnelIOInputs inputs) {}

  /**
   * Sets voltage of the Funnel motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the idle mode for the Funnel motor
   *
   * @param enable Sets brake mode on true, coast on false
   */
  public default void enableBrakeMode(boolean enable) {}
}
