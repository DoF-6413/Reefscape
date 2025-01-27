// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class ModuleIOSim implements ModuleIO {

  // Sim objects
  private final FlywheelSim m_driveSim;
  private final FlywheelSim m_turnSim;

  // PID FF controllers
  private final PIDController m_driveController;
  private final PIDController m_turnController;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private double driveSetpoint = 0.0;
  private double turnSetpoint = 0.0;

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
            DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD);
    m_turnController =
        new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
    m_driveFeedforward =
        new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV);

    // Configure controllers
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Drive and Turn based on setpoint
    double driveVoltage =
        m_driveController.calculate(inputs.driveVelocityRadPerSec, driveSetpoint)
            + m_driveFeedforward.calculate(driveSetpoint);
    double turnVoltage = m_turnController.calculate(inputs.turnAbsolutePositionRad, turnSetpoint);

    // Update simulated motors
    this.setDriveVoltage(driveVoltage);
    this.setTurnVoltage(turnVoltage);
    m_driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    m_turnSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update Drive motor inputs
    inputs.driveIsConnected = true;
    inputs.drivePositionRad +=
        m_driveSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVoltage = driveVoltage;
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
    inputs.turnAppliedVoltage = turnVoltage;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(double volts) {
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
    driveSetpoint = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnSetpoint = position.getRadians();
  }
}
