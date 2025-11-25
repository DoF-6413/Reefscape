package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.PathfindingCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController m_auxButtonBoard =
      new CommandXboxController(OperatorConstants.AUX_BUTTON_BOARD);
  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.AUX_XBOX_CONTROLLER);

  // Autos
  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
        // Real robot, instantiates hardware IO implementations
      case REAL:
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                new GyroIOPigeon2());
        break;
        // Sim robot, instantiates physics sim IO implementations
      case SIM:
        m_driveSubsystem =
            new Drive(
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new GyroIO() {});
        break;
        // Replayed robot, disables all IO implementations
      default:
        m_driveSubsystem =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        break;
    }

    /* Autonomous Routines */
    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // Leave
    m_autoChooser.addOption("Leave", AutoCommands.leave(m_driveSubsystem, 0.4, 4));

    /* Test Routines */
    m_autoChooser.addOption("2 Meter Test", new PathPlannerAuto("Forward"));

    /* Characterization Routines */
    m_autoChooser.addOption(
        "Drive Feedforward Characterization",
        DriveCommands.feedforwardCharacterization(m_driveSubsystem));
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(m_driveSubsystem));

    // Adds an "Auto" tab on ShuffleBoard
    Shuffleboard.getTab("Auto").add(m_autoChooser.getSendableChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    this.driverControllerBindings();
    this.auxButtonBoardBindings();
    this.auxControllerBindings();
  }

  /** Driver Controls */
  private void driverControllerBindings() {
    /* Driving the robot */
    // Default to field relative driving
    m_driveSubsystem.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> 0.8 * -m_driverController.getRightX())
            .withName("FieldRelativeDrive"));
    // Field relative
    m_driverController
        .rightStick()
        .onTrue(
            DriveCommands.fieldRelativeDrive(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> 0.8 * -m_driverController.getRightX())
                .withName("FieldRelativeDrive"));
    // Lock robot heading to 0 degrees
    m_driverController
        .povUp()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> Rotation2d.fromRadians(0))
                .withName("0DegreeHeadingDrive"));
    // Lock robot heading to 90 degrees
    m_driverController
        .povLeft()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> Rotation2d.fromRadians(Math.PI / 2))
                .withName("90DegreeHeadingDrive"));
    // Lock robot heading to 180 degrees
    m_driverController
        .povDown()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> Rotation2d.fromRadians(Math.PI))
                .withName("180DegreeHeadingDrive"));
    // Lock robot heading to -90 degrees
    m_driverController
        .povRight()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> Rotation2d.fromRadians(-Math.PI / 2))
                .withName("-90DegreeHeadingDrive"));
    // Lock forward/backward movement
    m_driverController
        .start()
        .onTrue(
            DriveCommands.fieldRelativeDrive(
                    m_driveSubsystem,
                    () -> 0.0,
                    () -> -m_driverController.getLeftX(),
                    () -> -m_driverController.getRightX())
                .withName("FieldRelativeDriveNoX"));

    /* Gyro */
    // Reset Gyro heading, making the front side of the robot the new 0 degree angle
    // m_driverController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(() -> m_driveSubsystem.zeroYaw(), m_driveSubsystem)
    //             .withName("ZeroYaw"));
    // Front reset
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(
                    () ->
                        m_driveSubsystem.resetPose(
                            new Pose2d(
                                m_driveSubsystem.getCurrentPose2d().getTranslation(),
                                RobotStateConstants.isRed()
                                    ? Rotation2d.k180deg
                                    : Rotation2d.kZero)),
                    m_driveSubsystem)
                .ignoringDisable(true)
                .withName("ZeroYaw"));
    // Back reset
    m_driverController
        .y()
        .onTrue(
            new InstantCommand(
                    () ->
                        m_driveSubsystem.resetPose(
                            new Pose2d(
                                m_driveSubsystem.getCurrentPose2d().getTranslation(),
                                RobotStateConstants.isRed()
                                    ? Rotation2d.kZero
                                    : Rotation2d.k180deg)),
                    m_driveSubsystem)
                .ignoringDisable(true)
                .withName("ZeroYaw"));

    // Closest CORAL STATION
    m_driverController
        .leftBumper()
        .onTrue(
            PathfindingCommands.driveToClosestCoralStation(
                m_driveSubsystem, 0, m_driverController.leftBumper().negate()));

    /* Misc */
    // Stop in X
    m_driverController
        .b()
        // .whileTrue(new InstantCommand(() -> m_driveSubsystem.stopWithX(), m_driveSubsystem));
        .onTrue(
            Commands.runOnce(
                () ->
                    System.out.println(
                        FieldConstants.APRILTAG_FIELD_LAYOUT
                            .getTagPose(22)
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(new Translation2d(5.39, 3.17)))));
  }

  /** Aux Button Board Controls */
  public void auxButtonBoardBindings() {
    /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    /* Score */
    m_auxButtonBoard
        .axisLessThan(OperatorConstants.BUTTON_BOARD.SCORE.BUTTON_ID, -0.5)
        .onTrue(
            new InstantCommand(
                () -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED),
                m_CEESubsystem))
        .onFalse(
            SuperstructureCommands.setSpeeds(
                m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem, 0, 0, 0));

    /* CORAL and ALGAE */
    // L1 or PROCESSOR
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.L1_PROCESSOR.BUTTON_ID)
        .onTrue(SuperstructureCommands.positionsToL1(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(
            m_auxButtonBoard.axisGreaterThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID,
                0.5)) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.positionsToProcessor(
                m_periscopeSubsystem, m_algaePivotSubsystem));
    // L2 CORAL or ALGAE
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.L2.BUTTON_ID)
        .onTrue(
            SuperstructureCommands.positionsToL2Coral(
                m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(
            m_auxButtonBoard.axisGreaterThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID,
                0.5)) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.intakeL2Algae(
                    m_periscopeSubsystem,
                    m_algaePivotSubsystem,
                    m_AEESubsystem,
                    m_CEESubsystem,
                    m_funnelSubsystem)
                .andThen(
                    new InstantCommand(
                        () -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
                        m_AEESubsystem)));
    // L3 CORAL or ALGAE
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.L3.BUTTON_ID)
        .onTrue(
            SuperstructureCommands.positionsToL3Coral(
                m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(
            m_auxButtonBoard.axisGreaterThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID,
                0.5)) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.intakeL3Algae(
                    m_periscopeSubsystem,
                    m_algaePivotSubsystem,
                    m_AEESubsystem,
                    m_CEESubsystem,
                    m_funnelSubsystem)
                .andThen(
                    new InstantCommand(
                        () -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
                        m_AEESubsystem)));
    // L4 or NET
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.L4_NET.BUTTON_ID)
        .onTrue(
            SuperstructureCommands.positionsToL4(
                m_periscopeSubsystem, m_algaePivotSubsystem, m_CEESubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(
            m_auxButtonBoard.axisGreaterThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID,
                0.5)) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.positionsToNet(
                m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem));
    // Ground ALGAE
    m_auxButtonBoard
        .axisGreaterThan(OperatorConstants.BUTTON_BOARD.GROUND_ALGAE.BUTTON_ID, 0.5)
        .onTrue(
            SuperstructureCommands.intakeGroundAlgae(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                    m_periscopeSubsystem,
                    m_algaePivotSubsystem,
                    m_AEESubsystem,
                    m_CEESubsystem,
                    m_funnelSubsystem)
                .andThen(
                    Commands.runOnce(
                        () -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
                        m_AEESubsystem)));
    // Adjust Periscope Height
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.CLIMB_DEPLOY.BUTTON_ID)
        .and(m_auxButtonBoard.axisLessThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5))
        .onTrue(
            new InstantCommand(
                () -> m_periscopeSubsystem.adjustHeight(Units.inchesToMeters(1)),
                m_periscopeSubsystem));
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.CLIMB_RETRACT.BUTTON_ID)
        .and(m_auxButtonBoard.axisLessThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5))
        .onTrue(
            new InstantCommand(
                () -> m_periscopeSubsystem.adjustHeight(Units.inchesToMeters(-1)),
                m_periscopeSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Climb ~~~~~~~~~~~~~~~~~~~~ */
    // Deploy Climber
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.CLIMB_DEPLOY.BUTTON_ID)
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5))
        .onTrue(
            new InstantCommand(
                () ->
                    m_climberSubsystem.setVoltage(
                        RobotStateConstants.MAX_VOLTAGE * ClimberConstants.DEPLOY_PERCENT_SPEED),
                m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // Retract Climber
    m_auxButtonBoard
    .button(OperatorConstants.BUTTON_BOARD.CLIMB_DEPLOY.BUTTON_ID)
    .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5))
        .onTrue(
            new InstantCommand(
                () ->
                    m_climberSubsystem.setVoltage(
                        RobotStateConstants.MAX_VOLTAGE *
    ClimberConstants.RETRACT_PERCENT_SPEED),
                m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Pathfinding Selection ~~~~~~~~~~~~~~~~~~~~ */
    // REEF Face AB
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_AB.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "A", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "A")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToA"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "B", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "B")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToB"));
    // REEF Face CD
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_CD.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "C", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "C")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToC"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "D", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "D")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToD"));
    // REEF Face EF
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_EF.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "F", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "F")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToF"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "E", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "E")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToE"));
    // REEF Face GH
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_GH.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "H", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "H")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToH"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "G", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "G")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToG"));
    // REEF Face IJ
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_IJ.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "J", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "J")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToJ"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "I", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "I")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToI"));
    // REEF Face KL
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_KL.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "K", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "K")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToK"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            // PathfindingCommands.driveToBranch(
            //         m_driveSubsystem, "L", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
            PathfindingCommands.alignToBranch(m_driveSubsystem, "L")
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToL"));
  }

  /** Aux Xbox Controls */
  public void auxControllerBindings() {
    // AEE testing binding
    m_auxController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> m_AEESubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED),
                m_AEESubsystem))
        .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));
    m_auxController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
                m_AEESubsystem))
        .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));

    // CEE testing binding
    m_auxController
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED),
                m_CEESubsystem))
        .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));
    m_auxController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> m_CEESubsystem.setPercentSpeed(CEEConstants.INTAKE_PERCENT_SPEED),
                m_CEESubsystem))
        .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));

    // Funnel testing binding
    m_auxController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> m_funnelSubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED),
                m_funnelSubsystem))
        .onFalse(new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(0), m_funnelSubsystem));
    m_auxController
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> m_funnelSubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
                m_funnelSubsystem))
        .onFalse(new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(0), m_funnelSubsystem));

    // ALGAE Pivot testing binding
    m_auxController
        .y()
        .onTrue(
            new InstantCommand(
                () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.MAX_ANGLE_RAD),
                m_algaePivotSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
                m_algaePivotSubsystem));
    m_auxController
        .x()
        .onTrue(
            new InstantCommand(
                () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.MIN_ANGLE_RAD),
                m_algaePivotSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
                m_algaePivotSubsystem));

    // Periscope testing binding
    m_auxController
        .a()
        .onTrue(
            new InstantCommand(
                () -> m_periscopeSubsystem.setPosition(Units.inchesToMeters(18)),
                m_periscopeSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MIN_HEIGHT_M),
                m_periscopeSubsystem));
    m_auxController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  m_periscopeSubsystem.enablePID(false);
                  m_periscopeSubsystem.setVoltage(6);
                },
                m_periscopeSubsystem))
        .onFalse(
            new InstantCommand(() -> m_periscopeSubsystem.setVoltage(0), m_periscopeSubsystem));
    m_auxController
        .rightStick()
        .onTrue(
            new InstantCommand(() -> m_periscopeSubsystem.resetPosition(0), m_periscopeSubsystem));
    m_auxController
        .start()
        .onTrue(
            new InstantCommand(() -> m_periscopeSubsystem.enablePID(true), m_periscopeSubsystem));
    m_auxController
        .back()
        .onTrue(
            new InstantCommand(() -> m_periscopeSubsystem.enablePID(false), m_periscopeSubsystem));

    /* Climb */
    // Joystick to move
    m_climberSubsystem.setDefaultCommand(
        new InstantCommand(
            () ->
                m_climberSubsystem.setVoltage(
                    RobotStateConstants.MAX_VOLTAGE * m_auxController.getLeftY()),
            m_climberSubsystem));
    // Deploy
    m_auxController
        .povUp()
        .onTrue(new InstantCommand(() -> m_climberSubsystem.setVoltage(2), m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // Retract
    m_auxController
        .povDown()
        .onTrue(new InstantCommand(() -> m_climberSubsystem.setVoltage(-2), m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    // /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    // /* Score */
    // m_auxController
    //     .rightTrigger()
    //     .onTrue(SuperstructureCommands.score(m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

    // /* CORAL and ALGAE */
    // // L1 or PROCESSOR
    // m_auxController
    //     .a()
    //     .onTrue(SuperstructureCommands.positionsToL1(m_periscopeSubsystem,
    // m_algaePivotSubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
    //     .onTrue(
    //         SuperstructureCommands.positionsToProcessor(
    //             m_periscopeSubsystem, m_algaePivotSubsystem));
    // // L2 CORAL or ALGAE
    // m_auxController
    //     .x()
    //     .onTrue(
    //         SuperstructureCommands.positionsToL2Coral(
    //             m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
    //     .onTrue(
    //         SuperstructureCommands.intakeL2Algae(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem));
    // // L3 CORAL or ALGAE
    // m_auxController
    //     .b()
    //     .onTrue(
    //         SuperstructureCommands.positionsToL3Coral(
    //             m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
    //     .onTrue(
    //         SuperstructureCommands.intakeL3Algae(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem));
    // // L4 or NET
    // m_auxController
    //     .y()
    //     .onTrue(
    //         SuperstructureCommands.positionsToL4(
    //             m_periscopeSubsystem, m_algaePivotSubsystem, m_CEESubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
    //     .onTrue(
    //         SuperstructureCommands.positionsToNet(
    //             m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem));
    // // Ground ALGAE
    // m_auxController
    //     .rightBumper()
    //     .onTrue(
    //         SuperstructureCommands.intakeGroundAlgae(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem));
    // // CORAL Intake
    // m_auxController
    //     .leftTrigger()
    //     .onTrue(
    //         SuperstructureCommands.intakeCoral(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem));
    // // Zero mechanisms
    // m_auxController
    //     .back()
    //     .onTrue(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  /**
   * Sets all mechanisms to brake mode, intended for use when the robot is disabled.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void allMechanismsBrakeMode(boolean enable) {
    m_driveSubsystem.enableBrakeModeAll(enable);
  }
}
