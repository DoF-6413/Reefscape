// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CEE extends SubsystemBase {

  private final CEEIO io;
  private final CEEIOInputsAutoLogged inputs = new CEEIOInputsAutoLogged();

  /** Creates a new CEE. */
  public CEE(CEEIO CEEIO) {
    // initialize the CEE subsystem
    System.out.println("[Init] Creating CEE");
    this.io = CEEIO;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CEE", inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
