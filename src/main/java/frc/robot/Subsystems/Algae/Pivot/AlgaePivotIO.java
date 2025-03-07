package frc.robot.Subsystems.Algae.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {

  @AutoLog
  public static class AlgaePivotIOInputs {
    /** Voltage applied to the ALGAE Pivot motor in volts */
    public double appliedVoltage = 0.0;
    /** Current draws of the ALGAE Pivot motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the ALGAE Pivot motor in celsius */
    public double tempCelsius = 0.0;
    /** Angular position of the Pivot from the absolute encoder in radians */
    public double absPositionRad = 0.0;
    /** Velocity of the ALGAE Pivot in radians per second */
    public double velocityRadPerSec = 0.0;
  }

  /**
   * Periodically updates the logged inputs for the ALGAE Pivot motor.
   *
   * @param inputs Inputs from the auto logger.
   */
  public default void updateInputs(AlgaePivotIOInputs inputs) {}

  /**
   * Sets the idle mode of the ALGAE Pivot motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public default void enableBrakeMode(boolean enable) {}

  /**
   * Sets voltage of the ALGAE Pivot motor. The value inputed is clamped between values of -12 to
   * 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public default void setVoltage(double volts) {}
}
