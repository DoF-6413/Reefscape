// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ModuleIOSim implements ModuleIO {

  private FlywheelSim driveSim;
  private FlywheelSim steerSim;


  public ModuleIOSim(){
    System.out.println("[Init] Creating ModuleIOSim");
    
    driveSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), DriveConstants.DRIVE_MOI_KG_M2, DriveConstants.DRIVE_GEAR_RATIO),
      DCMotor.getKrakenX60(1), 1);

  }
}
