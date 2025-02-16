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

  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Creates a new Coral End Effector Subsystem instances.
   *
   * <p>This constructor creates a new CEE subsystem object with given IO implementation
   *
   * @param io CEEIO implementation of the current mode of the robot
   */
  public CEE(CEEIO io) {
    // Initialize the CEE subsystem
    System.out.println("[Init] Creating Coral End Effector");
    m_io = io;

    m_PIDController = new PIDController(CEEConstants.KP, CEEConstants.KI, CEEConstants.KD);

    // Tunable PID values
    SmartDashboard.putBoolean("PIDFF/CEE/KP", false);
    SmartDashboard.putNumber("PIDFF/CEE/KP", CEEConstants.KP);
    SmartDashboard.putNumber("PIDFF/CEE/KI", CEEConstants.KI);
    SmartDashboard.putNumber("PIDFF/CEE/KD", CEEConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("CEE", m_inputs);

    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }
  }

  /**
   * Sets voltage of the CEE motor
   *
   * @param volts A value between -12 (full reverse speed) tp 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the velocity, in radians per second, of the CEE PID controller
   *
   * @param setPoint Velocity in radians per second
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID values for the CEE motor's PID controller in code
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable PID for the CEE
   *
   * @param enable True to enable PID, false to disable
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }
}
