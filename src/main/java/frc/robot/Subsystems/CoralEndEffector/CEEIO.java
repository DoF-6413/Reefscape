package frc.robot.Subsystems.CoralEndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface CEEIO {

  @AutoLog
  public static class CEEIOInputs {
    /** Voltage applied to the CEE motor */
    public double appliedVoltage = 0.0;
    /** Current drawn by the CEE motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the CEE motor in celsius */
    public double tempCelsius = 0.0;
    /** Velocity of the CEE in radians per second */
    public double velocityRadPerSec = 0.0;
    /** If the beam break sensor is broken, an object is in between the sensor  */
    public boolean isbeamBreakTriggered = false;
  }

  /**
   * Updates the logged inputs for the CEE. Must be called periodically.
   *
   * @param inputs Inputs from the auto logger.
   */
  public default void updateInputs(CEEIOInputs inputs) {}

  /**
   * Sets the idle mode of the CEE motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public default void enableBrakeMode(boolean enable) {}

  /**
   * Sets voltage of the CEE motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public default void setVoltage(double volts) {}
}
