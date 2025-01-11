// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands.TeleopCommands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Gyro.Gyro;

public class DefaultDriveCommand extends Command {
    /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  Gyro gyro;

  int index = 1;
  int prevIndex = index;

  public DefaultDriveCommand(
    Drive drive,
    Gyro gyro,
    CommandXboxController controller,
    int startingIndex) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.gyro = gyro;
    this.controller = controller;
    index = startingIndex;

    addRequirements(drive);
  }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){}  



}
