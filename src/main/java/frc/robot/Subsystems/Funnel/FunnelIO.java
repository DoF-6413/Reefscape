// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {

@AutoLog
public static class FunnelIOInputs {
    /** Voltage that Funnel motor draws */
    public double appliedVoltage = 0.0;
    /** Position of the wheel in radians */
    public double positionRad = 0.0;
    /** Velocity of the wheel in radians per sec */
    public double velocityRadPerSec = 0.0;
    /** Current drawn by the motor in amps */
    public double currentAmps = 0.0;
    /** Temperature of the motor in celsius */
    public double tempCelsius = 0.0;
  }

  /**
   * Peridocially updates the logged inputs for the Module.
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(FunnelIOInputs inputs) {}

  /**
   * Manually sets voltage of the Funnel motor
   *
   * @param volts A value between -12 (full reverse) to 12 (full forward)
   */
  public default void setVoltage(double volts) {}
}