// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CEE extends SubsystemBase {
  private final CEEIO m_io;
  private final CEEIOInputsAutoLogged m_inputs = new CEEIOInputsAutoLogged();

  // PID controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new CORAL End Effector ({@link CEE}) instance.
   *
   * <p>This creates a new CEE {@link SubsystemBase} object with the given IO implementation which
   * determines whether the methods and inputs are initialized with the real, sim, or replay code.
   *
   * @param io {@link CEEIO} implementation of the current mode of the robot.
   */
  public CEE(CEEIO io) {
    System.out.println("[Init] Creating Coral End Effector");

    // Initialize the IO implementation
    m_io = io;

    // Initialize the PID controller
    m_PIDController = new PIDController(CEEConstants.KP, CEEConstants.KI, CEEConstants.KD);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/CEE/EnableTunung", false);
    SmartDashboard.putNumber("PIDFF_Tuning/CEE/KP", CEEConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/CEE/KI", CEEConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/CEE/KD", CEEConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("CEE", m_inputs);

    // Control the CEE through the PID controller if enabled, open loop voltage control if disabled
    if (m_enablePID) {
      this.setVoltage(m_PIDController.calculate(m_inputs.velocityRadPerSec));
    }

    // Enable and update tunable PID gains through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/CEE/EnableTuning", false)) {
      this.updatePID();
    }
  }

  /**
   * Sets the idle mode of the CEE motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the CEE motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the CEE PID controller.
   *
   * @param setpoint Velocity in radians per second.
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID gains for PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable closed loop PID control for the CEE.
   *
   * @param enable {@code true} to enable PID control, {@code false} to disable.
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the CEE motors from SmartDashboard inputs. */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (CEEConstants.KP != SmartDashboard.getNumber("PIDFF_Tuning/CEE/KP", CEEConstants.KP)
        || CEEConstants.KI != SmartDashboard.getNumber("PIDFF_Tuning/CEE/KI", CEEConstants.KI)
        || CEEConstants.KD != SmartDashboard.getNumber("PIDFF_Tuning/CEE/KD", CEEConstants.KD)) {
      CEEConstants.KP = SmartDashboard.getNumber("PIDFF_Tuning/CEE/KP", CEEConstants.KP);
      CEEConstants.KI = SmartDashboard.getNumber("PIDFF_Tuning/CEE/KI", CEEConstants.KI);
      CEEConstants.KD = SmartDashboard.getNumber("PIDFF_Tuning/CEE/KD", CEEConstants.KD);
      // Sets the new gains
      this.setPID(CEEConstants.KP, CEEConstants.KI, CEEConstants.KD);
    }
  }
}
