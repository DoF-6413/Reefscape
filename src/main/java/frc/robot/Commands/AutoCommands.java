package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.Drive;

public class AutoCommands {
<<<<<<< HEAD
  /**
   * Auto that uses selectable choosers for building an auto that uses Pathfinding. Up to 2 piece.
   *
   * @param drive {@link Drive} subsystem
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that runs the auto build from the options on the SmartDashboard
   *     choosers.
   */
  public static Command dynamicPathfindingAuto(
      Drive drive, Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    // Constants
    final double DELAY_BETWEEN_ACTIONS = 0.25;
    final double CORAL_STATION_TIMEOUT = 3;
    final double WALL_DISTANCE_M = 0;

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
    firstCoralLevel.addOption("L1", SuperstructureCommands.positionsToL1(periscope, algaePivot));
    firstCoralLevel.addDefaultOption(
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
            drive, FieldConstants.CORAL_STATION_POSES.get("CS1L"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS1C",
        PathfindingCommands.pathfindToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get("CS1C"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS1R",
        PathfindingCommands.pathfindToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get("CS1R"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2L",
        PathfindingCommands.pathfindToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get("CS2L"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2C",
        PathfindingCommands.pathfindToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get("CS2C"), WALL_DISTANCE_M, 0, false));
    coralStation.addOption(
        "CS2R",
        PathfindingCommands.pathfindToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get("CS2R"), WALL_DISTANCE_M, 0, false));
    LoggedDashboardChooser<Command> secondBranch = new LoggedDashboardChooser<>("Second BRANCH");
    secondBranch.addDefaultOption(
        "None (1.5P)",
        Commands.waitSeconds(15)
            .alongWith(Commands.print("1.5P").andThen(Commands.waitSeconds(1)).repeatedly()));
    secondBranch.addOption("L", PathfindingCommands.alignToBranch(drive, "L"));
    secondBranch.addOption("K", PathfindingCommands.alignToBranch(drive, "K"));
    secondBranch.addOption("A", PathfindingCommands.alignToBranch(drive, "A"));
    secondBranch.addOption("B", PathfindingCommands.alignToBranch(drive, "B"));
    secondBranch.addOption("C", PathfindingCommands.alignToBranch(drive, "C"));
    secondBranch.addOption("D", PathfindingCommands.alignToBranch(drive, "D"));
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
          // Update options
          startingPose.periodic();
          firstBranch.periodic();
          firstCoralLevel.periodic();
          coralStation.periodic();
          secondBranch.periodic();
          secondCoralLevel.periodic();

          // Reset odometry if not updated by Vision already
          if (drive.getCurrentPose2d().getX() == 0.0) {
            drive.resetPose(startingPose.get());
          }

          // Schedule the auto command
          Commands.parallel(
                  PathfindingCommands.alignToBranch(drive, firstBranch.get()),
                  firstCoralLevel.get(),
                  Commands.print("Aligning to first CORAL"))
              .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
              .andThen(
                  SuperstructureCommands.score(aee, cee, funnel)
                      .alongWith(Commands.print("Scoring first CORAL")))
              .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
              .andThen(
                  DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0)
                      .withTimeout(DELAY_BETWEEN_ACTIONS)) // TODO: add if necessary
              .andThen(
                  Commands.parallel(
                      coralStation.get(),
                      SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel)
                          // TODO: test w/o timeout
                          .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
                          .andThen(
                              SuperstructureCommands.intakeCoral(
                                  periscope,
                                  algaePivot,
                                  aee,
                                  cee,
                                  funnel)), // TODO: test w/o timeout
                      Commands.print("Going to CORAL STATION")))
              .andThen(
                  Commands.race(
                      Commands.waitSeconds(CORAL_STATION_TIMEOUT),
                      Commands.waitUntil(
                          () ->
                              cee.isBeamBreakExitTriggered()
                                  && !cee.isBeamBreakEntranceTriggered())))
              .andThen(
                  Commands.parallel(
                      secondBranch.get(),
                      secondCoralLevel.get(),
                      Commands.print("Aligning to second BRANCH")))
              .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS))
              .andThen(
                  SuperstructureCommands.score(aee, cee, funnel)
                      .alongWith(Commands.print("Scoring second CORAL")))
              .schedule();
        },
        drive);
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
  public static Command pathfindingAutoOnePiece(
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

    final int reefAprilTagID;
    if (branch == "A" || branch == "B") {
      reefAprilTagID = 18;
    } else if (branch == "C" || branch == "D") {
      reefAprilTagID = 17;
    } else if (branch == "E" || branch == "F") {
      reefAprilTagID = 22;
    } else if (branch == "G" || branch == "H") {
      reefAprilTagID = 21;
    } else if (branch == "I" || branch == "J") {
      reefAprilTagID = 20;
    } else {
      reefAprilTagID = 19;
    }

    return Commands.runOnce(
            () -> {
              // Update robot pose if it hasn't been updated by the Vision already
              if (drive.getCurrentPose2d().getX() == 0.0) {
                drive.resetPose(startingPose);
              }
            },
            drive)
        .andThen(
            Commands.sequence(
                    // Algin to the BRANCH and raise the Periscope
                    PathfindingCommands.pathfindToAprilTag(drive, reefAprilTagID, 0.75, true),
                    Commands.parallel(
                        PathfindingCommands.driveToBranch(drive, branch, 0).finishAtGoal(),
                        coralPosition.withTimeout(0.5)))
                .withTimeout(10) // TODO: Test timout with side autos
            )
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS))
        .andThen(
            // Stop and score the CORAL
            Commands.parallel(
                Commands.runOnce(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee),
                Commands.runOnce(() -> drive.stop(), drive)))
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS * 2))
        .andThen(
            // Move backward and zero the Superstructure to avoid touching the CORAL
            DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0, () -> 0)
                .withTimeout(TIME_BETWEEN_ACTIONS))
        .andThen(SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel));
  }

  /**
   * Two CORAL auto that uses vision auto alignment and PathPlanner's Pathfinder for driving
   * segments
   *
   * @param drive {@link Drive} subsystem
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param startingPose {@link Pose2d} of the start pose
   * @param pieces Number of CORAL to be scored (1 or 2)
   * @param branches BRANCHES to score at, in order
   * @param coralLevels CORAL Levels to score, in order
   * @param coralStationName CORAL STATION to go to
   * @return {@link Command} that runs the 1 or 2 Piece auto routine
   */
  public static Command pathfindingTwoPiece(
      Drive drive,
      Periscope periscope,
      AlgaePivot algaePivot,
      AEE aee,
      CEE cee,
      Funnel funnel,
      Pose2d startingPose,
      String[] branches,
      int[] coralLevels,
      String coralStationName) {
    final DriveToPose[] driveToBranches = new DriveToPose[2];
    final Command[] positionToCoral = new Command[2];
    final DriveToPose coralStation;

    for (int i = 0; i < 2; i++) {
      driveToBranches[i] = PathfindingCommands.driveToBranch(drive, branches[i], 0);
      switch (coralLevels[i]) {
        case 1:
          positionToCoral[i] = SuperstructureCommands.positionsToL1(periscope, algaePivot);
          break;

        case 2:
          positionToCoral[i] =
              SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee);
          break;

        case 3:
          positionToCoral[i] =
              SuperstructureCommands.positionsToL3Coral(periscope, algaePivot, aee);
          break;

        case 4:
          positionToCoral[i] = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
          break;

        default:
          positionToCoral[i] = SuperstructureCommands.positionsToL1(periscope, algaePivot);
          break;
      }
    }

    coralStation =
        PathfindingCommands.driveToFieldElement(
            drive, FieldConstants.CORAL_STATION_POSES.get(coralStationName), 0, 0, false);

    final int reefAprilTagID;
    if (branches[1] == "A" || branches[1] == "B") {
      reefAprilTagID = 18;
    } else if (branches[1] == "C" || branches[1] == "D") {
      reefAprilTagID = 17;
    } else if (branches[1] == "E" || branches[1] == "F") {
      reefAprilTagID = 22;
    } else if (branches[1] == "G" || branches[1] == "H") {
      reefAprilTagID = 21;
    } else if (branches[1] == "I" || branches[1] == "J") {
      reefAprilTagID = 20;
    } else {
      reefAprilTagID = 19;
    }

    // return Commands.runOnce(
    //         () -> {
    //           // Update robot pose if it hasn't been updated by the Vision already
    //           if (drive.getCurrentPose2d().getX() == 0.0) {
    //             drive.resetPose(startingPose);
    //           }
    //         },
    //         drive)
    //     .andThen(
    //         Commands.parallel(
    //             // Drive to the BRANCH and raise the Periscope
    //             driveToBranches[0].finishAtGoal(),
    //             positionToCoral[0].withTimeout(0.25).beforeStarting(Commands.waitSeconds(0.25))))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(
    //         Commands.run(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)
    //             .withTimeout(0.25))
    //     .andThen(
    //         DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0, () ->
    // 0).withTimeout(0.5))
    //     .andThen(
    //         Commands.parallel(
    //             coralStation,
    //             Commands.sequence(
    //                 SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel)
    //                     .withTimeout(0.25),
    //                 Commands.waitSeconds(0.5),
    //                 SuperstructureCommands.intakeCoral(periscope, algaePivot, aee, cee, funnel)
    //                     .withTimeout(0.25))))
    //     .andThen(
    //         Commands.race(
    //             Commands.waitUntil(() -> cee.isBeamBreakTriggered()), Commands.waitSeconds(3)))
    //     .andThen(
    //         Commands.parallel(
    //             Commands.runOnce(() -> funnel.setPercentSpeed(0), funnel),
    //             driveToBranches[1].finishAtGoal(),
    //             positionToCoral[1].withTimeout(0.25).beforeStarting(Commands.waitSeconds(0.25))))
    //     .andThen(Commands.waitSeconds(0.25))
    //     .andThen(
    //         Commands.runOnce(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)
    //             .withTimeout(0.25))
    //     .andThen(
    //         DriveCommands.robotRelativeDrive(drive, () -> 0.25, () -> 0, () ->
    // 0).withTimeout(0.5))
    //     .andThen(SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel));
    return AutoCommands.pathfindingAutoOnePiece(
            drive,
            periscope,
            algaePivot,
            aee,
            cee,
            funnel,
            startingPose,
            branches[0],
            coralLevels[0])
        .andThen(
            Commands.parallel(
                Commands.deadline(
                    Commands.waitUntil(
                        () ->
                            cee.isBeamBreakExitTriggered() && !cee.isBeamBreakEntranceTriggered()),
                    coralStation.finishAtGoal()),
                SuperstructureCommands.intakeCoral(periscope, algaePivot, aee, cee, funnel)))
        .andThen(
            Commands.sequence(
                PathfindingCommands.pathfindToAprilTag(drive, reefAprilTagID, 1.5, true),
                Commands.parallel(positionToCoral[1], driveToBranches[1].finishAtGoal())))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(Commands.runOnce(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0, () -> 0).withTimeout(0.5))
        .andThen(SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel));
  }

  /**
   * Auto that uses Pathfinding to score 1 piece and go to the nearest CORAL STATION.
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
  public static Command pathfindingAutoOneAndHalfPiece(
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
                    drive, branch, PathPlannerConstants.DEFAULT_WALL_DISTANCE_M),
                SuperstructureCommands.positionsToL4(periscope, algaePivot, cee)))
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS))
        .andThen(coralPosition)
        .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS))
        .andThen(
            PathfindingCommands.pathfindToClosestCoralStation(
                drive, PathPlannerConstants.DEFAULT_WALL_DISTANCE_M, () -> false));
  }

=======
>>>>>>> b9102cc76935759d1f8003713d68906981a93c13
  public static Command leave(Drive drive, double driveSpeed, double driveTime) {
    return Commands.runOnce(() -> drive.zeroYaw(), drive)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            Commands.parallel(
                DriveCommands.robotRelativeDrive(drive, () -> driveSpeed, () -> 0.0, () -> 0.0)
                    .withTimeout(driveTime)));
  }
}
