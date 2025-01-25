// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class ModuleIOSim implements ModuleIO {

  private FlywheelSim driveSim;
  private FlywheelSim steerSim;

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim");

    driveSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                DriveConstants.DRIVE_MOI_KG_M2,
                DriveConstants.DRIVE_GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            0);

    steerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), DriveConstants.TURN_MOI_KG_M2, DriveConstants.STEER_GEAR_RATIO),
            DCMotor.getNEO(1),
          0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    steerSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    double angleDiffRad =
        steerSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionRad =
        inputs.drivePositionRad
            + (driveSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC);
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVoltage = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turnPositionRad = turnRelativePositionRad;
    inputs.turnVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVoltage = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSim.setInputVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    steerSim.setInputVoltage(volts);
  }
}
