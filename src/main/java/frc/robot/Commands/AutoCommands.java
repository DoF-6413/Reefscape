package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.Algae.EndEffector.AEE;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.CoralEndEffector.CEE;
import frc.robot.Subsystems.CoralEndEffector.CEEConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Periscope.Periscope;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommands {
  public static Command dynamicPathfindingAuto(
      Drive drive, Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    // Constants
    final double DELAY_BETWEEN_ACTIONS = 0.25;
    final double CORAL_STATION_DELAY = 1;
    final double WALL_DISTANCE_M = 0.1;

    // Choosers to build the auto
    LoggedDashboardChooser<Pose2d> startingPose = new LoggedDashboardChooser<>("Starting Pose");
    startingPose.addOption("SLL", PathPlannerConstants.STARTING_LINE_LEFT);
    startingPose.addDefaultOption("SLC", PathPlannerConstants.STARTING_LINE_CENTER);
    startingPose.addOption("SLR", PathPlannerConstants.STARTING_LINE_RIGHT);
    LoggedDashboardChooser<String> firstBranch = new LoggedDashboardChooser<>("First BRANCH");
    firstBranch.addOption("E", "E");
    firstBranch.addOption("F", "F");
    firstBranch.addDefaultOption("G", "G");
    firstBranch.addOption("H", "H");
    firstBranch.addOption("I", "I");
    firstBranch.addOption("J", "J");
    LoggedDashboardChooser<Command> firstCoralLevel =
        new LoggedDashboardChooser<>("First CORAL Level");
    firstCoralLevel.addDefaultOption(
        "L1", SuperstructureCommands.positionsToL1(periscope, algaePivot));
    firstCoralLevel.addOption(
        "L2", SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee));
    firstCoralLevel.addOption(
        "L3", SuperstructureCommands.positionsToL3Coral(periscope, algaePivot, aee));
    firstCoralLevel.addOption(
        "L4", SuperstructureCommands.positionsToL4(periscope, algaePivot, cee));
    LoggedDashboardChooser<Command> coralStation = new LoggedDashboardChooser<>("CORAL STATION");
    coralStation.addDefaultOption(
        "None (1P)",
        Commands.waitSeconds(15).alongWith(Commands.repeatingSequence(Commands.print("1P"))));
    coralStation.addOption(
        "CS1L",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS1L"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS1C",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS1C"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS1R",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS1R"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2L",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS2L"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2C",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS2C"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2R",
        PathfindingCommands.pathfindToFieldElement(
            FieldConstants.CORAL_STATION_POSES.get("CS2R"), WALL_DISTANCE_M, 0, false));
    LoggedDashboardChooser<String> secondBranch = new LoggedDashboardChooser<>("Second BRANCH");
    secondBranch.addOption("L", "L");
    secondBranch.addOption("K", "K");
    secondBranch.addDefaultOption("A", "A");
    secondBranch.addOption("B", "B");
    secondBranch.addOption("C", "C");
    secondBranch.addOption("D", "D");
    LoggedDashboardChooser<Command> secondCoralLevel =
        new LoggedDashboardChooser<>("Second CORAL Level");
    secondCoralLevel.addOption("L1", SuperstructureCommands.positionsToL1(periscope, algaePivot));
    secondCoralLevel.addDefaultOption(
        "L2", SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee));
    secondCoralLevel.addOption(
        "L3", SuperstructureCommands.positionsToL3Coral(periscope, algaePivot, aee));
    secondCoralLevel.addOption(
        "L4", SuperstructureCommands.positionsToL4(periscope, algaePivot, cee));

    // Put choosers into Auto tab for easy accessibility
    Shuffleboard.getTab("Auto").add(startingPose.getSendableChooser());
    Shuffleboard.getTab("Auto").add(firstBranch.getSendableChooser());
    Shuffleboard.getTab("Auto").add(firstCoralLevel.getSendableChooser());
    Shuffleboard.getTab("Auto").add(coralStation.getSendableChooser());
    Shuffleboard.getTab("Auto").add(secondBranch.getSendableChooser());
    Shuffleboard.getTab("Auto").add(secondCoralLevel.getSendableChooser());

    // Initialize options within the choosers so it doesn't crash
    startingPose.periodic();
    firstBranch.periodic();
    firstCoralLevel.periodic();
    coralStation.periodic();
    secondBranch.periodic();
    secondCoralLevel.periodic();

    return Commands.runOnce(
            () -> {
              startingPose.periodic();
              firstBranch.periodic();
              firstCoralLevel.periodic();
              coralStation.periodic();
              secondBranch.periodic();
              secondCoralLevel.periodic();
              drive.resetPose(startingPose.get());
            },
            drive)
        .andThen(
            Commands.parallel(
                PathfindingCommands.pathfindToBranch(firstBranch.get(), WALL_DISTANCE_M),
                firstCoralLevel.get()))
        .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
        .andThen(SuperstructureCommands.score(aee, cee, funnel))
        .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
        .andThen(
            Commands.parallel(
                coralStation.get(),
                SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel)
                    .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
                    .andThen(
                        SuperstructureCommands.intakeCoral(
                            periscope, algaePivot, aee, cee, funnel))))
        .andThen(
            Commands.race(
                Commands.waitSeconds(CORAL_STATION_DELAY),
                Commands.waitUntil(() -> cee.isBeamBreakTriggered())))
        .andThen(
            Commands.parallel(
                PathfindingCommands.pathfindToBranch(secondBranch.get(), WALL_DISTANCE_M),
                secondCoralLevel.get()))
        .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
        .andThen(SuperstructureCommands.score(aee, cee, funnel));
  }

  /**
   * Auto that uses Pathfinding to score 1 piece.
   *
   * @param drive {@link Drive} subsystem
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param startingPose {@link Pose2d} of the starting position
   * @param branch BRANCH letter to Pathfind to.
   * @param coralLevel CORAL Level
   * @return {@link Command} that runs the 1 piece auto.
   */
  public static Command pathfindingAuto1Piece(
      Drive drive,
      Periscope periscope,
      AlgaePivot algaePivot,
      AEE aee,
      CEE cee,
      Funnel funnel,
      Pose2d startingPose,
      String branch,
      int coralLevel) {
    final double TIME_BETWEEN_ACTIONS = 0.5;
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

    return Commands.runOnce(() -> drive.resetPose(startingPose), drive)
        .andThen(
            Commands.parallel(
                PathfindingCommands.pathfindToBranch(
                    branch, PathPlannerConstants.DEFAULT_WALL_DISTANCE_M),
                SuperstructureCommands.positionsToL4(periscope, algaePivot, cee)))
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS).andThen(coralPosition));
  }

  public static Command deadreakon1Piece(
      Drive drive,
      Periscope periscope,
      AlgaePivot algaePivot,
      AEE aee,
      CEE cee,
      Funnel funnel,
      double driveSpeed,
      int coralLevel) {
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
    final double DRIVE_TIME_SEC = 4;
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
                Commands.runOnce(
                    () -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)));
  }
}
