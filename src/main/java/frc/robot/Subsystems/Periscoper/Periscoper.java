// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Periscoper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Periscoper extends SubsystemBase {

  private final PeriscoperIO m_io;
  private final PeriscoperIOInputsAutoLogged m_inputs = new PeriscoperIOInputsAutoLogged();

  /**
   * Constructs a new Periscope subsystem instance.
   * 
   * <p>This constructor creates a new Periscope subsystem object with given IO implementation
   * 
   * @param io PeriscoperIO implementation of the current robot mode
   */
  public Periscoper(PeriscoperIO io) {
    // initialize the Periscoper subsystem
    System.out.println("[Init] Creating Periscoper");
    this.m_io = io;

    // Tunable PID & Feedforward values
    SmartDashboard.putBoolean("PIDFF/Periscoper/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP);
    SmartDashboard.putNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI);
    SmartDashboard.putNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD);
    SmartDashboard.putNumber("PIDFF/Periscoper/KS", PeriscoperConstants.KS);
    SmartDashboard.putNumber("PIDFF/Periscoper/KV", PeriscoperConstants.KV);
    SmartDashboard.putNumber("PIDFF/Periscoper/KG", PeriscoperConstants.KG);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Periscoper", m_inputs);

    if (SmartDashboard.getBoolean("PIDFF/Periscoper/EnableTuning", false)) {
      this.updatePID();
      this.updateFF();
    }
  }

  /**
   * Sets voltage of the Periscoper motors
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the position of the Periscoper
   * 
   * @param heightMeters Height of the Periscoper in meters
   */
  public void setPosition(double heightMeters) {
    m_io.setPosition(heightMeters);
  }

  /**
   * Sets the PID gains of the Periscoper motors' built in closed loop controllers
   * 
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_io.setPID(kP, kI, kD);
  }

  /** Update PID gains for the Periscoper motors from SmartDashboard inputs */
  private void updatePID() {
    if (PeriscoperConstants.KP
            != SmartDashboard.getNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP)
        || PeriscoperConstants.KI
            != SmartDashboard.getNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI)
        || PeriscoperConstants.KD
            != SmartDashboard.getNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD)) {
      PeriscoperConstants.KP =
          SmartDashboard.getNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP);
      PeriscoperConstants.KI =
          SmartDashboard.getNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI);
      PeriscoperConstants.KD =
          SmartDashboard.getNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD);
      this.setPID(PeriscoperConstants.KP, PeriscoperConstants.KI, PeriscoperConstants.KD);
    }
  }

  /** Update FeedForward gains for the Periscoper motors from SmartDashboard inputs */
  private void updateFF() {
    if (PeriscoperConstants.KS
            != SmartDashboard.getNumber("PIDFF/Periscoper/KS", PeriscoperConstants.KS)
        || PeriscoperConstants.KV
            != SmartDashboard.getNumber("PIDFF/Periscoper/KV", PeriscoperConstants.KV)
        || PeriscoperConstants.KG
            != SmartDashboard.getNumber("PIDFF/Periscoper/KG", PeriscoperConstants.KG)) {
      PeriscoperConstants.KS =
          SmartDashboard.getNumber("PIDFF/Periscoper/KS", PeriscoperConstants.KS);
      PeriscoperConstants.KV =
          SmartDashboard.getNumber("PIDFF/Periscoper/KV", PeriscoperConstants.KV);
      PeriscoperConstants.KG =
          SmartDashboard.getNumber("PIDFF/Periscoper/KG", PeriscoperConstants.KG);
      this.setPID(PeriscoperConstants.KS, PeriscoperConstants.KV, PeriscoperConstants.KG);
    }
  }
}
