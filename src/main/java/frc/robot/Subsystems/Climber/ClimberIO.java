// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    /** Whether a signal is being recieved by the Climber motor or not */
    public boolean isConnected = false;
    /** Voltage applied to the Climber motor */
    public double appliedVoltage = 0.0;
    /** Current draw of the Climber motor in amps */
    public double currentAmps = 0.0;
    /** Tempature of the Climber motor in celsius */
    public double tempCelsius = 0.0;
    /** Angular position of the Climber in radians */
    public double positionRad = 0.0;
    /** Velocity of the Climber in radians per second */
    public double velocityRadPerSec = 0.0;
  }

  /**
   * Updates the logged inputs for the Climber. Must be called periodically.
   *
   * @param inputs Inputs from the auto logger.
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the idle mode of the Climber motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public default void enableBrakeMode(boolean enable) {}

  /**
   * Sets voltage of the Climber motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the idle mode for the Climber motor
   *
   * @param enable Sets brake mode on true, coast on false
   */
  public default void setBrakeMode(boolean enable) {}
}
