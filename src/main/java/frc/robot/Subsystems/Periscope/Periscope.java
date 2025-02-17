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
   * This constructs a new {@link Periscope} instance.
   *
   * <p>This creates a new Periscope {@link SubsystemBase} object with given IO implementation which
   * determines whether the methods and inputs are initailized with the real, sim, or replay code
   *
   * @param io {@link PeriscopeIO} implementation of the current robot mode
   */
  public Periscope(PeriscopeIO io) {
    System.out.println("[Init] Creating Periscope");

    // Initailize the IO implementation
    m_io = io;

    // Tunable PID & Feedforward values
    SmartDashboard.putBoolean("PIDFF_Tuning/Periscope/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV);
    SmartDashboard.putNumber("PIDFF_Tuning/Periscope/KA", PeriscopeConstants.KA);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Periscope", m_inputs);

    // Enable and update tunable PID values through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/Periscope/EnableTuning", false)) {
      this.updatePID();
      this.updateFF();
    }
  }

  /**
   * Sets voltage of the Periscope motors. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the position of the Periscope using a motion profiled PID controller
   *
   * @param heightMeters Position of the Periscope in meters
   */
  public void setPosition(double heightMeters) {
    m_io.setPosition(heightMeters);
  }

  /**
   * Sets the PID gains of the Periscope motors' Profiled PID controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_io.setPID(kP, kI, kD);
  }

  /**
   * Sets the Feedforward values for the Periscope motors' Feedforward controller
   *
   * @param kS Static gain value
   * @param kG Gravity gain value
   * @param kV Velocity gain value
   * @param kA Acceleration gain value
   */
  public void setFF(double kS, double kG, double kV, double kA) {
    m_io.setFF(kS, kG, kV, kA);
  }

  /** Update PID gains for the Periscope motors from SmartDashboard inputs */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (PeriscopeConstants.KP
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP)
        || PeriscopeConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI)
        || PeriscopeConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD)) {
      PeriscopeConstants.KP =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KP", PeriscopeConstants.KP);
      PeriscopeConstants.KI =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KI", PeriscopeConstants.KI);
      PeriscopeConstants.KD =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KD", PeriscopeConstants.KD);
      // Sets the new gains
      this.setPID(PeriscopeConstants.KP, PeriscopeConstants.KI, PeriscopeConstants.KD);
    }
  }

  /** Update Feedforward gains for the Periscope motors from SmartDashboard inputs */
  private void updateFF() {
    // If any value on SmartDashboard changes, update the gains
    if (PeriscopeConstants.KS
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS)
        || PeriscopeConstants.KV
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV)
        || PeriscopeConstants.KG
            != SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG)) {
      PeriscopeConstants.KS =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KS", PeriscopeConstants.KS);
      PeriscopeConstants.KG =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KG", PeriscopeConstants.KG);
      PeriscopeConstants.KV =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KV", PeriscopeConstants.KV);
      PeriscopeConstants.KV =
          SmartDashboard.getNumber("PIDFF_Tuning/Periscope/KA", PeriscopeConstants.KA);
      // Sets the new gains
      this.setFF(
          PeriscopeConstants.KS,
          PeriscopeConstants.KG,
          PeriscopeConstants.KV,
          PeriscopeConstants.KA);
    }
  }
}
