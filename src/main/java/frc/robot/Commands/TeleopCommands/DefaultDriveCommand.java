// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Gyro.Gyro;

public class DefaultDriveCommand extends Command {
  Drive m_drive;
  Gyro m_gyro;
  CommandXboxController m_controller;

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(Drive drive, Gyro gyro, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_gyro = gyro;
    m_controller = controller;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Normal Drive Mode */
    m_drive.driveWithDeadband(
        m_controller.getLeftX(), // Forward/backward (multiply by -1 bc controller axis inverted)
        -m_controller.getLeftY(), // Left/Right
        m_controller.getRightX()); // Rotation
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted whether the command was interrupted or finished
   */
  @Override
  public void end(boolean interrupted) {
    m_drive.driveWithDeadband(0, 0, 0);
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}
