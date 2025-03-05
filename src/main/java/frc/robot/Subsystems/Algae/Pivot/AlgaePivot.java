// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO m_io;
  private final AlgaePivotIOInputsAutoLogged m_inputs = new AlgaePivotIOInputsAutoLogged();

  // PID controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = true;

  /**
   * Constructs a new {@link AlgaePivot} instance.
   *
   * <p>This creates a new ALGAE Pivot {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link AlgaePivotIO} implementation of the current mode of the robot.
   */
  public AlgaePivot(AlgaePivotIO io) {
    System.out.println("[Init] Creating ALGAE Pivot");

    // Initialize IO implementation
    m_io = io;

    // Initialize PID Controller
    m_PIDController =
        new PIDController(
            RobotStateConstants.getMode() == RobotStateConstants.Mode.SIM
                ? AlgaePivotConstants.KP_SIM
                : AlgaePivotConstants.KP,
            RobotStateConstants.getMode() == RobotStateConstants.Mode.SIM
                ? AlgaePivotConstants.KI_SIM
                : AlgaePivotConstants.KI,
            RobotStateConstants.getMode() == RobotStateConstants.Mode.SIM
                ? AlgaePivotConstants.KD_SIM
                : AlgaePivotConstants.KD);
    m_PIDController.setTolerance(AlgaePivotConstants.ERROR_TOLERANCE_RAD);

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
      this.setVoltage(m_PIDController.calculate(m_inputs.positionRad));
    }

    // Enable and update tunable PID gains through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/ALGAE_Pivot/EnableTuning", false)) {
      this.updatePID();
    }
  }

  /**
   * Sets the idle mode of the ALGAE Pivot motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the ALGAE Pivot motor. The value inputed is clamped between values of -12 to
   * 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the ALGAE Pivot PID controller.
   *
   * @param setpoint Angle in radians.
   */
  public void setAngle(double setpoint) {
    Logger.recordOutput("Superstructure/Setpoints/ALGAEPivotAngle", setpoint);
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Whether or not the ALGAE Pivot is at its angle setpoint.
   *
   * @return {@code true} if at setpoint angle, {@code false} if not
   */
  public boolean atSetpointAngle() {
    return m_PIDController.atSetpoint();
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
   * Enable closed loop PID control for the ALGAE Pivot.
   *
   * @param enable {@code true} to enable PID control, {@code false} to disable.
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the ALGAE Pivot motor from SmartDashboard inputs. */
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
