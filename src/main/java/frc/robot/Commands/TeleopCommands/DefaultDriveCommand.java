// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Gyro.Gyro;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  CommandXboxController controller;

  Drive drive;
  Gyro gyro;

  public DefaultDriveCommand(Drive drive, Gyro gyro, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.gyro = gyro;
    this.controller = controller;

    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Normal Drive Mode */
    drive.driveWithDeadband(
        controller.getLeftY(), // Forward/backward
        -controller.getLeftX(), // Left/Right (multiply by -1 bc controller a())is inverted)
        -controller.getRightX()); // Rotation;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveWithDeadband(0, 0, 0);
  }
  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}
