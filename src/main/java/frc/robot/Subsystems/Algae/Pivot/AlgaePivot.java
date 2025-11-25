// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO m_io;
  private final AlgaePivotIOInputsAutoLogged m_inputs = new AlgaePivotIOInputsAutoLogged();

  // PID controller
  private final ProfiledPIDController m_PIDController;
  private final ArmFeedforward m_feedforward;
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
        new ProfiledPIDController(
            AlgaePivotConstants.KP,
            AlgaePivotConstants.KI,
            AlgaePivotConstants.KD,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S),
                Units.degreesToRadians(AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2)));
    m_PIDController.setTolerance(AlgaePivotConstants.ERROR_TOLERANCE_RAD);
    m_PIDController.setGoal(AlgaePivotConstants.DEFAULT_ANGLE_RAD);
    m_feedforward =
        new ArmFeedforward(AlgaePivotConstants.KS, AlgaePivotConstants.KG, AlgaePivotConstants.KV);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/ALGAE_Pivot/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KP", AlgaePivotConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KI", AlgaePivotConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KD", AlgaePivotConstants.KD);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KS", AlgaePivotConstants.KS);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KG", AlgaePivotConstants.KG);
    SmartDashboard.putNumber("PIDFF_Tuning/ALGAE_Pivot/KV", AlgaePivotConstants.KV);
    SmartDashboard.putNumber(
        "PIDFF_Tuning/ALGAE_Pivot/Max_Vel_Deg", AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S);
    SmartDashboard.putNumber(
        "PIDFF_Tuning/ALGAE_Pivot/Max_Accel_Deg", AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2);
    SmartDashboard.putBoolean("PIDFF_Tuning/ALGAE_Pivot/EnablePID", m_enablePID);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // // Update and log inputs // TODO: test to reduce loop time
    // m_io.updateInputs(m_inputs);
    // Logger.processInputs("Algae Pivot", m_inputs);

    // if (DriverStation.isDisabled()) {
    //   this.setAngle(m_inputs.absPositionRad);
    //   this.setVoltage(0);
    // }

    // // Control the ALGAE Pivot through the PID controller if enabled, open loop voltage control
    // if
    // // disabled
    // if (SmartDashboard.getBoolean("PIDFF_Tuning/ALGAE_Pivot/EnablePID", m_enablePID)) {
    //   // Calculate voltage based on PID controller
    //   this.setVoltage(
    //       m_PIDController.calculate(m_inputs.absPositionRad)
    //           + m_feedforward.calculate(
    //               m_PIDController.getSetpoint().position,
    // m_PIDController.getSetpoint().velocity));

    //   Logger.recordOutput(
    //       "Superstructure/Setpoints/ALGAEPivot/AtSetpointAngle", m_PIDController.atSetpoint());
    //   Logger.recordOutput(
    //       "Superstructure/Setpoints/ALGAEPivot/AtGoalState", m_PIDController.atGoal());

    //   // Enable and update tunable PID gains through SmartDashboard
    //   if (SmartDashboard.getBoolean("PIDFF_Tuning/ALGAE_Pivot/EnableTuning", false)) {
    //     this.updatePID();
    //     this.updateFF();
    //     this.updateConstraints();
    //   }
    // }
  }

  /**
   * Sets the idle mode of the ALGAE Pivot motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  public void resetRelativeEncoder() {
    m_io.resetRelativeEncoder();
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
    m_PIDController.setGoal(setpoint);
  }

  public void setSetpoint(TrapezoidProfile.State state) {
    Logger.recordOutput("Superstructure/Setpoints/ALGAEPivotAngle", state.position);
    m_PIDController.setGoal(state);
  }

  /**
   * Whether or not the ALGAE Pivot is at its angle setpoint.
   *
   * @return {@code true} if at setpoint angle, {@code false} if not
   */
  public boolean atSetpointAngle() {
    return m_PIDController.atSetpoint();
  }

  public boolean atGoalState() {
    return m_PIDController.atGoal();
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

  public void setFF(double kS, double kG, double kV) {
    m_feedforward.setKs(kS);
    m_feedforward.setKg(kG);
    m_feedforward.setKv(kV);
  }

  public void setConstraints(double vel, double accel) {
    m_PIDController.setConstraints(new TrapezoidProfile.Constraints(vel, accel));
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

  private void updateFF() {
    // If any value on SmartDashboard changes, update the gains
    if (AlgaePivotConstants.KS
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KS", AlgaePivotConstants.KS)
        || AlgaePivotConstants.KG
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KG", AlgaePivotConstants.KG)
        || AlgaePivotConstants.KV
            != SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/Kv", AlgaePivotConstants.KV)) {
      AlgaePivotConstants.KS =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KS", AlgaePivotConstants.KS);
      AlgaePivotConstants.KG =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KG", AlgaePivotConstants.KG);
      AlgaePivotConstants.KV =
          SmartDashboard.getNumber("PIDFF_Tuning/ALGAE_Pivot/KV", AlgaePivotConstants.KV);
      // Sets the new gains
      this.setFF(AlgaePivotConstants.KS, AlgaePivotConstants.KG, AlgaePivotConstants.KV);
    }
  }

  private void updateConstraints() {
    // If any value on SmartDashboard changes, update the gains
    if (AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S
            != SmartDashboard.getNumber(
                "PIDFF_Tuning/ALGAE_Pivot/Max_Vel_Deg", AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S)
        || AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2
            != SmartDashboard.getNumber(
                "PIDFF_Tuning/ALGAE_Pivot/Max_Accel_Deg",
                AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2)) {
      AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S =
          SmartDashboard.getNumber(
              "PIDFF_Tuning/ALGAE_Pivot/Max_Vel_Deg", AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S);
      AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2 =
          SmartDashboard.getNumber(
              "PIDFF_Tuning/ALGAE_Pivot/Max_Accel_Deg",
              AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2);
      // Sets the new gains
      this.setConstraints(
          Units.degreesToRadians(AlgaePivotConstants.MAX_VELOCITY_DEG_PER_S),
          Units.degreesToRadians(AlgaePivotConstants.MAX_ACCELERATION_DEG_PER_S2));
    }
  }
}
