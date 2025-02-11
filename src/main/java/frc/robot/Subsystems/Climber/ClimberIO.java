// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        //Voltage that climb motor draws
        public double climberAppliedVoltage = 0.0;
        //Position of the wheel in radians
        public double climberPositionRad = 0.0;
        //Velocity of the wheel in radians per sec
        public double climberVelocityRadPerSec = 0.0;
        //Current drawn by the motor in amps
        public double climberCurrentAmps = 0.0;
        //Temperature of the motor in celsius
        public double climberTempCelsius = 0.0;
        //If a signal is being recieved from the Climb motor
        public boolean climberIsConnected = false;
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
  public default void setClimberVoltage(double volts) {}

  /**
   * Sets the idle mode for the Climb motor
   *
   * @param enable Sets break mode on true, coast on false
   */
  public default void setClimberBrakeMode(boolean enable) {}

   /**
   * Sets the velocity of the Climb motor using the closed loop controller built into the TalonFX
   * speed controller
   *
   * @param velocityRadPerSec Velocity to set Climb motor to in radians per second
   */
  public default void setClimberVelocity(double velocityRadPerSec) {}

  
}
    
