// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class ModuleIOSim implements ModuleIO {
  // Sim objects
  private final FlywheelSim m_driveSim;
  private final FlywheelSim m_turnSim;

  // Motor voltages
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  // PID FF controllers
  private final PIDController m_driveController;
  private SimpleMotorFeedforward m_driveFeedForward;
  private double m_driveSetpoint = 0.0;

  /**
   * Constructs a new ModuleIOSim instance
   *
   * <p>This creates a new ModuleIO object that uses the simulated versions of the KrakenX60 and NEO
   * motors to run the Drive and Turn of the simulated Module
   */
  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim");

    // Initilize simulated motors
    m_driveSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                DriveConstants.DRIVE_MOI_KG_M2,
                DriveConstants.DRIVE_GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            0);

    m_turnSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), DriveConstants.TURN_MOI_KG_M2, DriveConstants.STEER_GEAR_RATIO),
            DCMotor.getNEO(1),
            0);

    // Initilize PID FF controllers
    m_driveController =
        new PIDController(
            DriveConstants.DRIVE_KP_SIM, DriveConstants.DRIVE_KI_SIM, DriveConstants.DRIVE_KD_SIM);
    m_driveFeedForward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS_SIM, DriveConstants.DRIVE_KV_SIM);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Drive and Turn based on setpoint
    m_driveAppliedVolts =
        m_driveController.calculate(inputs.driveVelocityRadPerSec, m_driveSetpoint)
            + m_driveFeedForward.calculate(m_driveSetpoint);

    // Update simulated motors
    this.setDriveVoltage(m_driveAppliedVolts);
    m_driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    m_turnSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update Drive motor inputs
    inputs.driveIsConnected = true;
    inputs.drivePositionRad +=
        m_driveSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVoltage = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());

    // Update Turn motor inputs
    inputs.absoluteEncoderIsConnected = true;
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            inputs.turnAbsolutePositionRad
                + (m_turnSim.getAngularVelocityRadPerSec()
                    * RobotStateConstants.LOOP_PERIODIC_SEC));
    inputs.turnPositionRad = inputs.turnAbsolutePositionRad;
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVoltage = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_turnAppliedVolts = volts;
    m_driveSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveSetpoint = velocityRadPerSec;
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    m_driveFeedForward = new SimpleMotorFeedforward(kS, kV);
  }
}
