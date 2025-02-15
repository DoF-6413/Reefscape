// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // Voltage that climb motor draws
    public double appliedVoltage = 0.0;
    // Position of the wheel in radians
    public double positionRad = 0.0;
    // Velocity of the wheel in radians per sec
    public double velocityRadPerSec = 0.0;
    // Current drawn by the motor in amps
    public double currentAmps = 0.0;
    // Temperature of the motor in celsius
    public double tempCelsius = 0.0;
    // If a signal is being recieved from the Climb motor
    public boolean isConnected = false;
  }

  /**
   * Peridocially updates the logged inputs for the climb motor.
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Manually sets voltage of the Climb motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the idle mode for the Climb motor
   *
   * @param enable Sets break mode on true, coast on false
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets the velocity of the Climb motor using the closed loop controller built into the TalonFX
   * speed controller
   *
   * @param velocityRadPerSec Velocity to set Climb motor to in radians per second
   */
  public default void setVelocity(double velocityRadPerSec) {}

  /**
   * Sets the position of the Climb motor using the closed loop controller built into the TalonFX
   * speed controller
   *
   * @param Position to set Climb motor to in radians per second
   */
  public default void setPosition(double Position) {}

  public default void setPID(double kP, double kI, double kD) {}
}
