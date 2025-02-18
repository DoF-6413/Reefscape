// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO m_io;
  private final AlgaePivotIOInputsAutoLogged m_inputs = new AlgaePivotIOInputsAutoLogged();

  // PID controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new {@link AlgaePivot} instance.
   *
   * <p>This creates a new ALGAE Pivot {@link SubsystemBase} object with given IO implementation
   * which determines whether the methods and inputs are initailized with the real, sim, or replay
   * code
   *
   * @param io {@link AlgaePivotIO} implementation of the current mode of the robot
   */
  public AlgaePivot(AlgaePivotIO io) {
    System.out.println("[Init] Creating ALGAE Pivot");

    // Initialize IO implementation
    m_io = io;

    // Initalize PID Controller
    m_PIDController =
        new PIDController(AlgaePivotConstants.KP, AlgaePivotConstants.KI, AlgaePivotConstants.KD);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/ALGAE_Pivot/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KP", AlgaePivotConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KI", AlgaePivotConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KD", AlgaePivotConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Algae Pivot", m_inputs);

    // Control the ALGAE Pivot through the PID controller if enabled, open loop voltage control if
    // disabled
    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }

    // Enable and update tunable PID gains through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/ALGAE_Pivot/EnableTuning", false)) {
      this.updatePID();
    }
  }

  /**
   * Sets the idle mode for the ALGAE Pivot motor
   *
   * @param enable Sets brake mode on true, coast on false
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the ALGAE Pivot motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the ALGAE Pivot PID controller
   *
   * @param setpoint Angle in radians
   */
  public void setSetpoint(double setPoint) {
    m_PIDController.setSetpoint(setPoint);
  }

  /**
   * Sets the PID gains for PID controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable closed loop PID control for the ALGAE Pivot
   *
   * @param enable True to enable PID, false to disable
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the ALGAE Pivot motors from SmartDashboard inputs */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (AlgaePivotConstants.KP
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KP", AlgaePivotConstants.KP)
        || AlgaePivotConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KI", AlgaePivotConstants.KI)
        || AlgaePivotConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KD", AlgaePivotConstants.KD)) {
      AlgaePivotConstants.KP =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KP", AlgaePivotConstants.KP);
      AlgaePivotConstants.KI =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KI", AlgaePivotConstants.KI);
      AlgaePivotConstants.KD =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KD", AlgaePivotConstants.KD);
      // Sets the new gains
      this.setPID(AlgaePivotConstants.KP, AlgaePivotConstants.KI, AlgaePivotConstants.KD);
    }
  }
}
