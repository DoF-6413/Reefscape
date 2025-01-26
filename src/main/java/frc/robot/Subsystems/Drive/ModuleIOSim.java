// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class ModuleIOSim implements ModuleIO {

  // Sim objects
  private FlywheelSim driveSim;
  private FlywheelSim steerSim;

  /**
   * Constructs a new ModuleIOSim instance
   *
   * <p>This creates a new ModuleIO object that uses the simulated versions of the KrakenX60 and NEO
   * motors to run the Drive and Turn of the simulated Module
   */
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
    // Update simulated motors
    driveSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    steerSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update Drive motor inputs
    inputs.drivePositionRad +=
        driveSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update Turn motor inputs
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            inputs.turnAbsolutePositionRad
                + (steerSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC));
    inputs.turnPositionRad = inputs.turnAbsolutePositionRad;
    inputs.turnVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
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
