package frc.robot.Subsystems.Algae.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {

  @AutoLog
  public static class AlgaePivotIOInputs {
    /** Voltage applied to the Algae Pivot motor in volts */
    public double appliedVoltage = 0.0;
    /** Current draws of the Algae Pivot motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the Algae Pivot motor in celsius */
    public double tempCelsius = 0.0;
    /** Angular position of the Pivot in radians */
    public double positionRad = 0.0;
    /** Velocity of the Algae Pivot in radians per second */
    public double velocityRadPerSec = 0.0;
    /** Whether a signal is being received by the Algae Pivot motor or not */
    public boolean isConnected = false;
  }

  /**
   * Periodically updates the logged inputs for the Algae Pivot motor
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(AlgaePivotIOInputs inputs) {}

  /**
   * Sets voltage of the Algae Pivot motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the idle mode for the Algae Pivot motor
   *
   * @param enable Sets brake mode on true, coast on false
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets the position of the Pivot using a PID controller
   *
   * @param positionRad Angular position of the Pivot in radians
   */
  public default void setPosition(double positionRad) {}

  /**
   * Sets the PID gains of the Algae Pivot motor's PID controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public default void setPID(double kP, double kI, double kD) {}
}
