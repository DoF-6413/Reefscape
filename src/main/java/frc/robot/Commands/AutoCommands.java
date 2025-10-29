package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Algae.EndEffector.AEE;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.CoralEndEffector.CEE;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Funnel.Funnel;

public class AutoCommands {
  /**
   * Auto that uses selectable choosers for building an auto that uses Pathfinding. Up to 2 piece.
   *
   * @param drive {@link Drive} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that runs the auto build from the options on the SmartDashboard
   *     choosers.
   */
  public static void dynamicPathfindingAuto(
      Drive drive, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    // Constants
    final double DELAY_BETWEEN_ACTIONS = 0.25;
    final double CORAL_STATION_TIMEOUT = 3;
    final double WALL_DISTANCE_M = 0;
  }

  // Choosers to build the auto
  /**
   * LoggedDashboardChooser<Pose2d> startingPose = new LoggedDashboardChooser<>("Starting Pose");
   * startingPose.addOption("SLL", PathPlannerConstants.STARTING_LINE_LEFT);
   * startingPose.addDefaultOption("SLC", PathPlannerConstants.STARTING_LINE_CENTER);
   * startingPose.addOption("SLR", PathPlannerConstants.STARTING_LINE_RIGHT);
   * LoggedDashboardChooser<Command> firstBranch = new LoggedDashboardChooser<>("First BRANCH");
   * firstBranch.addOption("E", PathfindingCommands.alignToBranch(drive, "E"));
   * firstBranch.addOption("F", PathfindingCommands.alignToBranch(drive, "F"));
   * firstBranch.addDefaultOption("G", PathfindingCommands.alignToBranch(drive, "G"));
   * firstBranch.addOption("H", PathfindingCommands.alignToBranch(drive, "H"));
   * firstBranch.addOption("I", PathfindingCommands.alignToBranch(drive, "I"));
   * firstBranch.addOption("J", PathfindingCommands.alignToBranch(drive, "J"));
   * LoggedDashboardChooser<Command> firstCoralLevel = new LoggedDashboardChooser<>("First CORAL
   * Level"); firstCoralLevel.addOption("L1", SuperstructureCommands.positionsToL1(periscope,
   * algaePivot)); firstCoralLevel.addDefaultOption( "L2",
   * SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee));
   * firstCoralLevel.addOption( "L3", SuperstructureCommands.positionsToL3Coral(periscope,
   * algaePivot, aee)); firstCoralLevel.addOption( "L4",
   * SuperstructureCommands.positionsToL4(periscope, algaePivot, cee));
   * LoggedDashboardChooser<Command> coralStation = new LoggedDashboardChooser<>("CORAL STATION");
   * coralStation.addDefaultOption( "None (1P)",
   * Commands.waitSeconds(15).alongWith(Commands.repeatingSequence(Commands.print("1P"))));
   * coralStation.addOption( "CS1L", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS1L"), WALL_DISTANCE_M, 0, false));
   * coralStation.addOption( "CS1C", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS1C"), WALL_DISTANCE_M, 0, false));
   * coralStation.addOption( "CS1R", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS1R"), WALL_DISTANCE_M, 0, false));
   * coralStation.addOption( "CS2L", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS2L"), WALL_DISTANCE_M, 0, false));
   * coralStation.addOption( "CS2C", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS2C"), WALL_DISTANCE_M, 0, false));
   * coralStation.addOption( "CS2R", PathfindingCommands.pathfindToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get("CS2R"), WALL_DISTANCE_M, 0, false));
   * LoggedDashboardChooser<Command> secondBranch = new LoggedDashboardChooser<>("Second BRANCH");
   * secondBranch.addDefaultOption( "None (1.5P)", Commands.waitSeconds(15)
   * .alongWith(Commands.print("1.5P").andThen(Commands.waitSeconds(1)).repeatedly()));
   * secondBranch.addOption("L", PathfindingCommands.alignToBranch(drive, "L"));
   * secondBranch.addOption("K", PathfindingCommands.alignToBranch(drive, "K"));
   * secondBranch.addOption("A", PathfindingCommands.alignToBranch(drive, "A"));
   * secondBranch.addOption("B", PathfindingCommands.alignToBranch(drive, "B"));
   * secondBranch.addOption("C", PathfindingCommands.alignToBranch(drive, "C"));
   * secondBranch.addOption("D", PathfindingCommands.alignToBranch(drive, "D"));
   * LoggedDashboardChooser<Command> secondCoralLevel = new LoggedDashboardChooser<>("Second CORAL
   * Level"); secondCoralLevel.addOption("L1", SuperstructureCommands.positionsToL1(periscope,
   * algaePivot)); secondCoralLevel.addDefaultOption( "L2",
   * SuperstructureCommands.positionsToL2Coral(periscope, algaePivot, aee));
   * secondCoralLevel.addOption( "L3", SuperstructureCommands.positionsToL3Coral(periscope,
   * algaePivot, aee)); secondCoralLevel.addOption( "L4",
   * SuperstructureCommands.positionsToL4(periscope, algaePivot, cee));
   */

  // Put choosers into Auto tab for easy accessibility
  /**
   * Shuffleboard.getTab("Auto").add(startingPose.getSendableChooser());
   * Shuffleboard.getTab("Auto").add(firstBranch.getSendableChooser());
   * Shuffleboard.getTab("Auto").add(firstCoralLevel.getSendableChooser());
   * Shuffleboard.getTab("Auto").add(coralStation.getSendableChooser());
   * Shuffleboard.getTab("Auto").add(secondBranch.getSendableChooser());
   * Shuffleboard.getTab("Auto").add(secondCoralLevel.getSendableChooser());
   */

  // Initialize options within the choosers so it doesn't crash
  /**
   * startingPose.periodic(); firstBranch.periodic(); firstCoralLevel.periodic();
   * coralStation.periodic(); secondBranch.periodic(); secondCoralLevel.periodic();
   */
  /**
   * return Commands.runOnce( () -> { // Update options startingPose.periodic();
   * firstBranch.periodic(); firstCoralLevel.periodic(); coralStation.periodic();
   * secondBranch.periodic(); secondCoralLevel.periodic();
   *
   * <p>var robotPose = drive.getCurrentPose2d();
   *
   * <p>// Reset odometry if not updated by Vision already if (robotPose.getX() == 0.0) {
   * drive.resetPose(startingPose.get()); }
   *
   * <p>// Set starting angle drive.resetPose(new Pose2d(robotPose.getTranslation(),
   * startingPose.get().getRotation()));
   *
   * <p>// Schedule the auto command Commands.parallel( firstBranch.get(), firstCoralLevel.get(),
   * Commands.print("Aligning to first CORAL"))
   * .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS)) .andThen(
   * SuperstructureCommands.score(aee, cee, funnel) .alongWith(Commands.print("Scoring first
   * CORAL"))) .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS)) .andThen(
   * DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0)
   * .withTimeout(DELAY_BETWEEN_ACTIONS)) .andThen( Commands.parallel( coralStation.get(),
   * SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel) // TODO: test w/o timeout
   * irl .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS)) .andThen(
   * SuperstructureCommands.intakeCoral( periscope, algaePivot, aee, cee, funnel)), // TODO: test
   * w/o timeout irl Commands.print("Going to CORAL STATION"))) .andThen( Commands.race(
   * Commands.waitSeconds(CORAL_STATION_TIMEOUT), Commands.waitUntil( () ->
   * cee.isBeamBreakExitTriggered() && !cee.isBeamBreakEntranceTriggered()))) .andThen(
   * Commands.parallel( secondBranch.get(), secondCoralLevel.get(), Commands.print("Aligning to
   * second BRANCH"))) .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS)) .andThen(
   * SuperstructureCommands.score(aee, cee, funnel) .alongWith(Commands.print("Scoring second
   * CORAL"))) .andThen(Commands.waitSeconds(DELAY_BETWEEN_ACTIONS)) .andThen(
   * DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0)
   * .withTimeout(DELAY_BETWEEN_ACTIONS)) .andThen(SuperstructureCommands.zero(periscope,
   * algaePivot, aee, cee, funnel)) .schedule(); }, drive);
   *
   * <p>}
   */

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
  /**
   * public static Command pathfindingAutoOnePiece( Drive drive, Periscope periscope, AlgaePivot
   * algaePivot, AEE aee, CEE cee, Funnel funnel, Pose2d startingPose, String branch, int
   * coralLevel) { final double TIME_BETWEEN_ACTIONS = 0.5; final Command coralPosition; switch
   * (coralLevel) { case 1: coralPosition = SuperstructureCommands.positionsToL1(periscope,
   * algaePivot); break;
   *
   * <p>case 2: coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 3: coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 4: coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
   * break;
   *
   * <p>default: coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   * }
   */
  /**
   * return Commands.runOnce( () -> { var robotPose = drive.getCurrentPose2d();
   *
   * <p>// Reset odometry if not updated by Vision already if (robotPose.getX() == 0.0) {
   * drive.resetPose(startingPose); }
   *
   * <p>// Set starting angle drive.resetPose(new Pose2d(robotPose.getTranslation(),
   * startingPose.getRotation())); }, drive) .andThen( Commands.sequence( // Algin to the BRANCH and
   * raise the Periscope Commands.parallel( PathfindingCommands.alignToBranch(drive, branch),
   * coralPosition.withTimeout(0.5))) .withTimeout(10) // TODO: Test timout with side autos )
   * .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS)) .andThen( // Stop and score the CORAL
   * Commands.parallel( Commands.runOnce(() ->
   * cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee), Commands.runOnce(() ->
   * drive.stop(), drive))) .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS * 2)) .andThen( //
   * Move backward and zero the Superstructure to avoid touching the CORAL
   * DriveCommands.robotRelativeDrive(drive, () -> -0.5, () -> 0, () -> 0)
   * .withTimeout(TIME_BETWEEN_ACTIONS)) .andThen(SuperstructureCommands.zero(periscope, algaePivot,
   * aee, cee, funnel)); }
   */

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
  /**
   * public static Command pathfindingTwoPiece( Drive drive, Periscope periscope, AlgaePivot
   * algaePivot, AEE aee, CEE cee, Funnel funnel, Pose2d startingPose, String[] branches, int[]
   * coralLevels, String coralStationName) { final DriveToPose[] driveToBranches = new
   * DriveToPose[2]; final Command[] positionToCoral = new Command[2]; final DriveToPose
   * coralStation;
   *
   * <p>for (int i = 0; i < 2; i++) { driveToBranches[i] = PathfindingCommands.driveToBranch(drive,
   * branches[i], 0); switch (coralLevels[i]) { case 1: positionToCoral[i] =
   * SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   *
   * <p>case 2: positionToCoral[i] = SuperstructureCommands.positionsToL2Coral(periscope,
   * algaePivot, aee); break;
   *
   * <p>case 3: positionToCoral[i] = SuperstructureCommands.positionsToL3Coral(periscope,
   * algaePivot, aee); break;
   *
   * <p>case 4: positionToCoral[i] = SuperstructureCommands.positionsToL4(periscope, algaePivot,
   * cee); break;
   *
   * <p>default: positionToCoral[i] = SuperstructureCommands.positionsToL1(periscope, algaePivot);
   * break; } }
   */
  /**
   * coralStation = PathfindingCommands.driveToFieldElement( drive,
   * FieldConstants.CORAL_STATION_POSES.get(coralStationName), 0, 0, false);
   */

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
  /**
   * return AutoCommands.pathfindingAutoOnePiece( drive, periscope, algaePivot, aee, cee, funnel,
   * startingPose, branches[0], coralLevels[0]) .andThen( Commands.parallel( Commands.deadline(
   * Commands.waitUntil( () -> cee.isBeamBreakExitTriggered() &&
   * !cee.isBeamBreakEntranceTriggered()), coralStation.finishAtGoal()),
   * SuperstructureCommands.intakeCoral(periscope, algaePivot, aee, cee, funnel)))
   * .andThen(Commands.parallel(positionToCoral[1], driveToBranches[1].finishAtGoal()))
   * .andThen(Commands.waitSeconds(0.5)) .andThen(Commands.runOnce(() ->
   * cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee))
   * .andThen(Commands.waitSeconds(0.5)) .andThen( DriveCommands.robotRelativeDrive(drive, () ->
   * -0.5, () -> 0, () -> 0).withTimeout(0.5)) .andThen(SuperstructureCommands.zero(periscope,
   * algaePivot, aee, cee, funnel));
   *
   * <p>}
   */

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
  /**
   * public static Command pathfindingAutoOneAndHalfPiece( Drive drive, Periscope periscope,
   * AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel, Pose2d startingPose, String branch, int
   * coralLevel) { final double TIME_BETWEEN_ACTIONS = 0.5; final Command coralPosition; switch
   * (coralLevel) { case 1: coralPosition = SuperstructureCommands.positionsToL1(periscope,
   * algaePivot); break;
   *
   * <p>case 2: coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 3: coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 4: coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
   * break;
   *
   * <p>default: coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   * }
   */
  /**
   * return Commands.runOnce(() -> drive.resetPose(startingPose), drive) .andThen(
   * Commands.parallel( PathfindingCommands.pathfindToBranch( drive, branch,
   * PathPlannerConstants.DEFAULT_WALL_DISTANCE_M), SuperstructureCommands.positionsToL4(periscope,
   * algaePivot, cee))) .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS)) .andThen(coralPosition)
   * .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS)) .andThen(
   * PathfindingCommands.pathfindToClosestCoralStation( drive,
   * PathPlannerConstants.DEFAULT_WALL_DISTANCE_M, () -> false)); }
   */
  public static Command leave(Drive drive, double driveSpeed, double driveTime) {
    return Commands.runOnce(() -> drive.zeroYaw(), drive)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            Commands.parallel(
                DriveCommands.robotRelativeDrive(drive, () -> driveSpeed, () -> 0.0, () -> 0.0)
                    .withTimeout(driveTime)));
  }

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
  /**
   * public static Command deadreckonOnePiece( Drive drive, Periscope periscope, AlgaePivot
   * algaePivot, AEE aee, CEE cee, Funnel funnel, double driveSpeed, int coralLevel) { final double
   * DRIVE_TIME_SEC = 4; final Command coralPosition; switch (coralLevel) { case 1: coralPosition =
   * SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   *
   * <p>case 2: coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 3: coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 4: coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
   * break;
   *
   * <p>default: coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   * }
   *
   * <p>return Commands.runOnce( () -> // Update robot heading drive.resetPose( new Pose2d(
   * drive.getCurrentPose2d().getTranslation(), RobotStateConstants.isRed() ? Rotation2d.kZero :
   * Rotation2d.k180deg)), drive) .andThen(Commands.waitSeconds(0.5)) .andThen( Commands.parallel(
   * // Drive and raise the Periscope DriveCommands.fieldRelativeDriveAtAngle( drive, () ->
   * RobotStateConstants.isRed() ? -driveSpeed : driveSpeed, () -> 0, () ->
   * RobotStateConstants.isRed() ? Rotation2d.kZero : Rotation2d.k180deg), coralPosition)
   * .withDeadline(Commands.waitSeconds(DRIVE_TIME_SEC))) .andThen( Commands.parallel( // Stop and
   * score Commands.runOnce(() -> drive.setRaw(0, 0, 0), drive), Commands.run(() ->
   * cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)) .withTimeout(1))
   * .andThen(Commands.waitSeconds(1)) .andThen( Commands.sequence( Commands.runOnce(() ->
   * cee.setPercentSpeed(0), cee), // Drive back to avoid CORAL
   * DriveCommands.robotRelativeDrive(drive, () -> -driveSpeed, () -> 0, () -> 0) .withTimeout(1)))
   * .andThen( Commands.parallel( SuperstructureCommands.zero(periscope, algaePivot, aee, cee,
   * funnel), DriveCommands.robotRelativeDrive(drive, () -> 0, () -> 0, () -> 0))); }
   */

  /**
   * 1.5 Piece auto for scoring a specified CORAL on the G or H BRANCHES. Uses deadre
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
  /**
   * public static Command unethicalOneAndHalfPiece( Drive drive, Periscope periscope, AlgaePivot
   * algaePivot, AEE aee, CEE cee, Funnel funnel, double driveSpeed, int coralLevel) { final double
   * DRIVE_TIME_SEC = 4; final double TIME_BETWEEN_ACTIONS = 1; final Command coralPosition; switch
   * (coralLevel) { case 1: coralPosition = SuperstructureCommands.positionsToL1(periscope,
   * algaePivot); break;
   *
   * <p>case 2: coralPosition = SuperstructureCommands.positionsToL2Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 3: coralPosition = SuperstructureCommands.positionsToL3Coral(periscope, algaePivot,
   * aee); break;
   *
   * <p>case 4: coralPosition = SuperstructureCommands.positionsToL4(periscope, algaePivot, cee);
   * break;
   *
   * <p>default: coralPosition = SuperstructureCommands.positionsToL1(periscope, algaePivot); break;
   * }
   *
   * <p>return Commands.runOnce(() -> drive.zeroYaw(), drive) .andThen( Commands.parallel(
   * DriveCommands.fieldRelativeDriveAtAngle( drive, () -> driveSpeed, () -> 0, () ->
   * Rotation2d.kZero) .withTimeout(DRIVE_TIME_SEC), coralPosition)) .andThen( Commands.parallel(
   * Commands.runOnce(() -> drive.setRaw(0, 0, 0), drive), Commands.runOnce(() ->
   * cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee)))
   * .andThen(Commands.waitSeconds(TIME_BETWEEN_ACTIONS)) .andThen( Commands.parallel(
   * PathfindingCommands.pathfindToClosestCoralStation( drive,
   * PathPlannerConstants.DEFAULT_WALL_DISTANCE_M, () -> false),
   * SuperstructureCommands.zero(periscope, algaePivot, aee, cee, funnel) .andThen(
   * Commands.waitSeconds(TIME_BETWEEN_ACTIONS) .andThen( SuperstructureCommands.intakeCoral(
   * periscope, algaePivot, aee, cee, funnel))))); }
   */
}
