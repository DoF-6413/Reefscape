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
<<<<<<< feat#24-update-readme

  /**
   * 1 Piece auto for scoring a specified CORAL on the G or H BRANCHES. Doesn't use Vision (only
   * percent speed of the DT) to move the robot.
   *
   * @param drive {@link Drive} subsystem
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param driveSpeed Percent speed of the Drivetrain
   * @param coralLevel CORAL level to score
   * @return {@link Command} that runs the deadreckoned 1 piece auto.
   */
  public static Command deadreckonOnePiece(
      Drive drive,
      Periscope periscope,
      AlgaePivot algaePivot,
      AEE aee,
      CEE cee,
      Funnel funnel,
      double driveSpeed,
      int coralLevel) {
    final double DRIVE_TIME_SEC = 4;
    final Command coralPosition;
    switch (coralLevel) {
      case 1:
        coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot);
        break;

      case 2:
        coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee);
        break;

      case 3:
        coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot, aee);
        break;

      case 4:
        coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
        break;

      default:
        coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot);
        break;
    }

    return Commands.runOnce(() -> drive.zeroYaw(), drive)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            Commands.parallel(
                    DriveCommands.fieldRelativeDriveAtAngle(
                        drive,
                        () -> RobotStateConstants.isRed() ? -driveSpeed : driveSpeed,
                        () -> 0,
                        () -> Rotation2d.kZero),
                    coralPosition)
                .withDeadline(Commands.waitSeconds(DRIVE_TIME_SEC)))
        .andThen(
            Commands.runOnce(() -> drive.setRaw(0, 0, 0), drive)
                .alongWith(
                    Commands.run(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)
                        .withTimeout(1)))
        .andThen(Commands.waitSeconds(1))
        .andThen(
            DriveCommands.fieldRelativeDrive(
                    drive,
                    () -> RobotStateConstants.isRed() ? driveSpeed : -driveSpeed,
                    () -> 0,
                    () -> 0)
                .withTimeout(2));
  }

  /**
   * 1.5 Piece auto for scoring a specified CORAL on the G or H BRANCHES. Uses dead reckoning
   *
   * @param drive {@link Drive} subsystem
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param driveSpeed Percent speed of the Drivetrain
   * @param coralLevel CORAL level to score
   * @return {@link Command} that runs the dead reckoned 1 piece auto.
   */
  public static Command unethicalOneAndHalfPiece(
      Drive drive,
      Periscope periscope,
      AlgaePivot algaePivot,
      AEE aee,
      CEE cee,
      Funnel funnel,
      double driveSpeed,
      int coralLevel) {
    final double DRIVE_TIME_SEC = 4;
    final double TIME_BETWEEN_ACTIONS = 1;
    final Command coralPosition;
    switch (coralLevel) {
      case 1:
        coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot);
        break;

      case 2:
        coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee);
        break;

      case 3:
        coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot, aee);
        break;

      case 4:
        coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
        break;

      default:
        coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot);
        break;
    }

    return Commands.runOnce(() -> drive.zeroYaw(), drive)
        .andThen(
            Commands.parallel(
                DriveCommands.fieldRelativeDriveAtAngle(
                        drive, () -> driveSpeed, () -> 0, () -> Rotation2d.kZero)
                    .withTimeout(DRIVE_TIME_SEC),
                coralPosition))
        .andThen(
            Commands.parallel(
                Commands.runOnce(() -> drive.setRaw(0, 0, 0), drive),
                Commands.runOnce(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)))
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS))
        .andThen(
            Commands.parallel(
                PathfindingCommands.pathfindToClosestCoralStation(
                    drive, PathPlannerConstants.DEFAULT_WALL_DISTANCE_M, () -> false),
                SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel)
                    .andThen(
                        Commands.waitSeconds(TIME_BETWEEN_ACTIONS)
                            .andThen(
                                SuperstructureCommands.intakeCoral(
                                    periscope, algaePivot, aee, cee, funnel)))));
  }
=======
>>>>>>> Dev
}
