// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeEndEffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AEE extends SubsystemBase {
  private final AEEIO m_io;
  private final AEEIOInputsAutoLogged m_inputs = new AEEIOInputsAutoLogged();

  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new Algae End Effector subsystem instance.
   *
   * <p>This constructor creates a new AEE subsystem object with given IO implementation
   *
   * @param io AEEIO implementation of the current mode of the robot
   */
  public AEE(AEEIO io) {
    System.out.println("[Init] Creating Algae End Effector");
    m_io = io;

    m_PIDController = new PIDController(AEEConstants.KP, AEEConstants.KP, AEEConstants.KP);

    // Tunable PID values
    SmartDashboard.putBoolean("PIDFF/AEE/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/AEE/KP", AEEConstants.KP);
    SmartDashboard.putNumber("PIDFF/AEE/KI", AEEConstants.KI);
    SmartDashboard.putNumber("PIDFF/AEE/KD", AEEConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("AEE", m_inputs);

    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }
  }

  /**
   * Sets voltage of the AEE motor
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the AEE PID controller
   *
   * @param setpoint Velocity in radians per second
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID values for the Turn motor's built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable PID for the AEE
   *
   * @param enable True to enable PID, false to disable
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }
}
