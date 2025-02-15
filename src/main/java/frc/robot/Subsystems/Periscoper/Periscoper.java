// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Periscoper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Periscoper extends SubsystemBase {

  private final PeriscoperIO io;
  private final PeriscoperIOInputsAutoLogged inputs = new PeriscoperIOInputsAutoLogged();

  /** Creates a new Periscoper. */
  public Periscoper(PeriscoperIO periscoperIO) {
    // initialize the Periscoper subsystem
    System.out.println("[Init] Creating Periscoper");
    this.io = periscoperIO;

    SmartDashboard.putBoolean("PIDFF/Periscoper/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP);
    SmartDashboard.putNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI);
    SmartDashboard.putNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Periscoper", inputs);

    if (SmartDashboard.getBoolean("PIDFF/Periscoper/EnableTuning", false)) {
      this.updatePeriscoperPID();
      ;
    }
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setPosition(double heightMeters) {
    io.setPosition(heightMeters);
  }

  public void setPID(double kP, double kI, double kD) {

    io.setPID(kP, kI, kD);
  }

  private void updatePeriscoperPID() {
    if (PeriscoperConstants.KP
            != SmartDashboard.getNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP)
        || PeriscoperConstants.KI
            != SmartDashboard.getNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI)
        || PeriscoperConstants.KD
            != SmartDashboard.getNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD)) {
      PeriscoperConstants.KP =
          SmartDashboard.getNumber("PIDFF/Periscoper/KP", PeriscoperConstants.KP);
      PeriscoperConstants.KI =
          SmartDashboard.getNumber("PIDFF/Periscoper/KI", PeriscoperConstants.KI);
      PeriscoperConstants.KD =
          SmartDashboard.getNumber("PIDFF/Periscoper/KD", PeriscoperConstants.KD);
      this.setPID(PeriscoperConstants.KP, PeriscoperConstants.KI, PeriscoperConstants.KD);
    }
  }
}
