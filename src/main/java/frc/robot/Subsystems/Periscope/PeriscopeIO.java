package frc.robot.Subsystems.Periscope;

import org.littletonrobotics.junction.AutoLog;

public interface PeriscopeIO {

  @AutoLog
  public static class PeriscopeIOInputs {
    /** Applied voltage to the Periscope motors in volts */
    public double[] appliedVolts = {0.0, 0.0};
    /** Current draw of the Periscope motors in amps */
    public double[] currentDraw = {0.0, 0.0};
    /** Tempature of the Periscope motors in celsius */
    public double[] tempCelsius = {0.0, 0.0};
    /** Height of the Periscope in meters */
    public double heightMeters = 0.0;
    /** Linear velocity of the Periscope in meters */
    public double velocityMetersPerSec = 0.0;
    /** Whether a singal is being recieved by the Periscope motors or not */
    public boolean[] isConnected = {false, false};
  }

  /**
   * Updates the logged inputs for the Periscope. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(PeriscopeIOInputs inputs) {}

  /**
   * Sets voltage of the Periscope motors
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the position of the Periscope using the motors' closed loop controller built into the
   * TalonFX speed controller
   *
   * @param heightMeters Position of the Periscope in meters
   */
  public default void setPosition(double heightMeters) {}

  /**
   * Sets the PID gains of the Periscope motors' built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public default void setPID(double kP, double kI, double kD) {}

  /**
   * Sets the Feedforward values for the Periscope motors' built in closed loop controller
   *
   * @param kS Static gain value
   * @param kG Gravity gain value
   * @param kV Velocity gain value
   * @param kA Acceleration gain value
   */
  public default void setFF(double kS, double kG, double kV, double kA) {}
}
