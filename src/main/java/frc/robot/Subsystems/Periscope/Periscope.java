// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Periscope;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Periscope extends SubsystemBase {

  private final PeriscopeIO m_io;
  private final PeriscopeIOInputsAutoLogged m_inputs = new PeriscopeIOInputsAutoLogged();

  /**
   * Constructs a new Periscope subsystem instance.
   * 
   * <p>This constructor creates a new Periscope subsystem object with given IO implementation
   * 
   * @param io PeriscopeIO implementation of the current robot mode
   */
  public Periscope(PeriscopeIO io) {
    // initialize the Periscope subsystem
    System.out.println("[Init] Creating Periscope");
    this.m_io = io;

    // Tunable PID & Feedforward values
    SmartDashboard.putBoolean("PIDFF/Periscope/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/Periscope/KP", PeriscopeConstants.KP);
    SmartDashboard.putNumber("PIDFF/Periscope/KI", PeriscopeConstants.KI);
    SmartDashboard.putNumber("PIDFF/Periscope/KD", PeriscopeConstants.KD);
    SmartDashboard.putNumber("PIDFF/Periscope/KS", PeriscopeConstants.KS);
    SmartDashboard.putNumber("PIDFF/Periscope/KV", PeriscopeConstants.KV);
    SmartDashboard.putNumber("PIDFF/Periscope/KG", PeriscopeConstants.KG);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Periscope", m_inputs);

    if (SmartDashboard.getBoolean("PIDFF/Periscope/EnableTuning", false)) {
      this.updatePID();
      this.updateFF();
    }
  }

  /**
   * Sets voltage of the Periscope motors
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the position of the Periscope
   * 
   * @param heightMeters Height of the Periscope in meters
   */
  public void setPosition(double heightMeters) {
    m_io.setPosition(heightMeters);
  }

  /**
   * Sets the PID gains of the Periscope motors' built in closed loop controllers
   * 
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_io.setPID(kP, kI, kD);
  }

  /** Update PID gains for the Periscope motors from SmartDashboard inputs */
  private void updatePID() {
    if (PeriscopeConstants.KP
            != SmartDashboard.getNumber("PIDFF/Periscope/KP", PeriscopeConstants.KP)
        || PeriscopeConstants.KI
            != SmartDashboard.getNumber("PIDFF/Periscope/KI", PeriscopeConstants.KI)
        || PeriscopeConstants.KD
            != SmartDashboard.getNumber("PIDFF/Periscope/KD", PeriscopeConstants.KD)) {
      PeriscopeConstants.KP =
          SmartDashboard.getNumber("PIDFF/Periscope/KP", PeriscopeConstants.KP);
      PeriscopeConstants.KI =
          SmartDashboard.getNumber("PIDFF/Periscope/KI", PeriscopeConstants.KI);
      PeriscopeConstants.KD =
          SmartDashboard.getNumber("PIDFF/Periscope/KD", PeriscopeConstants.KD);
      this.setPID(PeriscopeConstants.KP, PeriscopeConstants.KI, PeriscopeConstants.KD);
    }
  }

  /** Update FeedForward gains for the Periscope motors from SmartDashboard inputs */
  private void updateFF() {
    if (PeriscopeConstants.KS
            != SmartDashboard.getNumber("PIDFF/Periscope/KS", PeriscopeConstants.KS)
        || PeriscopeConstants.KV
            != SmartDashboard.getNumber("PIDFF/Periscope/KV", PeriscopeConstants.KV)
        || PeriscopeConstants.KG
            != SmartDashboard.getNumber("PIDFF/Periscope/KG", PeriscopeConstants.KG)) {
      PeriscopeConstants.KS =
          SmartDashboard.getNumber("PIDFF/Periscope/KS", PeriscopeConstants.KS);
      PeriscopeConstants.KV =
          SmartDashboard.getNumber("PIDFF/Periscope/KV", PeriscopeConstants.KV);
      PeriscopeConstants.KG =
          SmartDashboard.getNumber("PIDFF/Periscope/KG", PeriscopeConstants.KG);
      this.setPID(PeriscopeConstants.KS, PeriscopeConstants.KV, PeriscopeConstants.KG);
    }
  }
}
