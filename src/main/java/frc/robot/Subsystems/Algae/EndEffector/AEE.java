// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Algae.EndEffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AEE extends SubsystemBase {
  private final AEEIO m_io;
  private final AEEIOInputsAutoLogged m_inputs = new AEEIOInputsAutoLogged();

  // PID Controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new ALGAE End Effector ({@link AEE}) instance.
   *
   * <p>This creates a new AEE {@link SubsystemBase} object with given IO implementation which
   * determines whether the methods and inputs are initialized with the real, sim, or replay code.
   *
   * @param io {@link AEEIO} implementation of the current mode of the robot.
   */
  public AEE(AEEIO io) {
    System.out.println("[Init] Creating ALGAE End Effector");

    // Initialize the IO implementation
    m_io = io;

    // Initialize the PID controller
    m_PIDController = new PIDController(AEEConstants.KP, AEEConstants.KI, AEEConstants.KD);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/AEE/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/AEE/KP", AEEConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/AEE/KI", AEEConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/AEE/KD", AEEConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("AEE", m_inputs);

    // Control the AEE through the PID controller if enabled, open loop voltage control if disabled
    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }

    // Enable and update tunable PID gains through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/AEE/EnableTuning", false)) {
      this.updatePID();
    }
  }

  /**
   * Sets the idle mode of the AEE motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the AEE motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the AEE PID controller.
   *
   * @param setpoint Velocity in radians per second.
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the gains for the PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable closed loop PID control for the AEE.
   *
   * @param enable {@code true} to enable PID control, {@code false} to disable.
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the AEE motor from SmartDashboard inputs. */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (AEEConstants.KP != SmartDashboard.getNumber("PIDFF_Tuning/AEE/KP", AEEConstants.KP)
        || AEEConstants.KI != SmartDashboard.getNumber("PIDFF_Tuning/AEE/KI", AEEConstants.KI)
        || AEEConstants.KD != SmartDashboard.getNumber("PIDFF_Tuning/AEE/KD", AEEConstants.KD)) {
      AEEConstants.KP = SmartDashboard.getNumber("PIDFF_Tuning/AEE/KP", AEEConstants.KP);
      AEEConstants.KI = SmartDashboard.getNumber("PIDFF_Tuning/AEE/KI", AEEConstants.KI);
      AEEConstants.KD = SmartDashboard.getNumber("PIDFF_Tuning/AEE/KD", AEEConstants.KD);
      // Sets the new gains
      this.setPID(AEEConstants.KP, AEEConstants.KI, AEEConstants.KD);
    }
  }
}
