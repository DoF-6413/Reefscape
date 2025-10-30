package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.Drive;

public class AutoCommands {
  public static Command leave(Drive drive, double driveSpeed, double driveTime) {
    return Commands.runOnce(() -> drive.zeroYaw(), drive)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            Commands.parallel(
                DriveCommands.robotRelativeDrive(drive, () -> driveSpeed, () -> 0.0, () -> 0.0)
                    .withTimeout(driveTime)));
  }
}