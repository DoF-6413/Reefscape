// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Periscoper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Periscoper extends SubsystemBase {

  private final PeriscoperIO io;
  private final PeriscoperIOInputsAutoLogged inputs = new PeriscoperIOInputsAutoLogged();

  /** Creates a new Periscoper. */
  public Periscoper(PeriscoperIO periscoperIO) {
    //initialize the Periscoper subsystem
    System.out.println("[Init] Creating Periscoper");
    this.io = periscoperIO;


  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Periscoper", inputs);
 
  }

  public void setVoltage(double volts){
    io.setVoltage(volts);
  }
  public void setVelocity(double velocity){
    io.setVelocity(velocity);
}
}
