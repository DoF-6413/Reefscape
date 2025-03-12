package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.PathfindingCommands;
import frc.robot.Commands.SuperstructureCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Algae.EndEffector.*;
import frc.robot.Subsystems.Algae.Pivot.*;
import frc.robot.Subsystems.Climber.*;
import frc.robot.Subsystems.CoralEndEffector.*;
import frc.robot.Subsystems.Drive.*;
import frc.robot.Subsystems.Funnel.*;
import frc.robot.Subsystems.Periscope.*;
import frc.robot.Subsystems.Vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;

  // Mechanisms
  private final AlgaePivot m_algaePivotSubsystem;
  private final Periscope m_periscopeSubsystem;
  private final Climber m_climberSubsystem;
  private final Funnel m_funnelSubsystem;
  private final AEE m_AEESubsystem;
  private final CEE m_CEESubsystem;

  // Utils
  private final Vision m_visionSubsystem;

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
        m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSparkMax());
        m_periscopeSubsystem = new Periscope(new PeriscopeIOTalonFX());
        m_climberSubsystem = new Climber(new ClimberIOTalonFX());
        m_funnelSubsystem = new Funnel(new FunnelIOSparkMax());
        m_AEESubsystem = new AEE(new AEEIOSparkMax() {});
        m_CEESubsystem = new CEE(new CEEIOSparkMax());
        m_visionSubsystem =
            new Vision(
                m_driveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.CAMERA.FRONT.CAMERA_INDEX),
                new VisionIOPhotonVision(VisionConstants.CAMERA.BACK.CAMERA_INDEX));
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
        m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSim());
        m_periscopeSubsystem = new Periscope(new PeriscopeIOSim());
        m_climberSubsystem = new Climber(new ClimberIOSim());
        m_funnelSubsystem = new Funnel(new FunnelIOSim());
        m_AEESubsystem = new AEE(new AEEIOSim() {});
        m_CEESubsystem = new CEE(new CEEIOSim());
        m_visionSubsystem =
            new Vision(
                m_driveSubsystem::addVisionMeasurement,
                // new VisionIOSim(
                //     VisionConstants.CAMERA.FRONT.CAMERA_INDEX,
                // m_driveSubsystem::getCurrentPose2d),
                // new VisionIOSim(
                //     VisionConstants.CAMERA.BACK.CAMERA_INDEX,
                // m_driveSubsystem::getCurrentPose2d))
                new VisionIO() {});
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
        m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIO() {});
        m_periscopeSubsystem = new Periscope(new PeriscopeIO() {});
        m_climberSubsystem = new Climber(new ClimberIO() {});
        m_funnelSubsystem = new Funnel(new FunnelIO() {});
        m_AEESubsystem = new AEE(new AEEIO() {});
        m_CEESubsystem = new CEE(new CEEIO() {});
        m_visionSubsystem = new Vision(m_driveSubsystem::addVisionMeasurement, new VisionIO() {});
        break;
    }

    /* PathPlanner Commands */
    NamedCommands.registerCommand(
        "Zero_Superstructure",
        SuperstructureCommands.zero(
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem));
    NamedCommands.registerCommand(
        "Position_L1",
        SuperstructureCommands.positionsToL1(m_periscopeSubsystem, m_algaePivotSubsystem));
    NamedCommands.registerCommand(
        "Position_L2_CORAL",
        SuperstructureCommands.positionsToL2Coral(
            m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem));
    NamedCommands.registerCommand(
        "Position_L3_CORAL",
        SuperstructureCommands.positionsToL3Coral(
            m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem));
    NamedCommands.registerCommand(
        "Position_L4",
        SuperstructureCommands.positionsToL4(
            m_periscopeSubsystem, m_algaePivotSubsystem, m_CEESubsystem));
    NamedCommands.registerCommand(
        "Score", SuperstructureCommands.score(m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));
    NamedCommands.registerCommand(
        "Intake_CORAL",
        SuperstructureCommands.intakeCoral(
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem));
    NamedCommands.registerCommand(
        "CEE_Out",
        Commands.runOnce(() -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED)));

    /* Autonomous Routines */
    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // Dynamic/Pathfinding Autos
    m_autoChooser.addOption(
        "Dynamic Pathfinding Auto",
        AutoCommands.dynamicPathfindingAuto(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem));
    m_autoChooser.addOption(
        "1P_SLC-G4 (Pathfinding)",
        AutoCommands.pathfindingAutoOnePiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            PathPlannerConstants.STARTING_LINE_CENTER,
            "G",
            4));
    m_autoChooser.addOption(
        "Deadreckon 1P L1",
        AutoCommands.deadreckonOnePiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            0.4,
            1));
    m_autoChooser.addOption(
        "Deadreckon 1P L2",
        AutoCommands.deadreckonOnePiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            0.4,
            2));
    m_autoChooser.addOption(
        "Deadreckon 1P L3",
        AutoCommands.deadreckonOnePiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            0.4,
            3));
    m_autoChooser.addOption(
        "Deadreckon 1P L4",
        AutoCommands.deadreckonOnePiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            0.4,
            4));
    m_autoChooser.addOption(
        "Unethical 1.5P L4",
        AutoCommands.unethicalOneAndHalfPiece(
            m_driveSubsystem,
            m_periscopeSubsystem,
            m_algaePivotSubsystem,
            m_AEESubsystem,
            m_CEESubsystem,
            m_funnelSubsystem,
            0.4,
            4));
    // 1 Piece
    // Starting Line Left (SLL)
    m_autoChooser.addOption("1P_SLL-IJ1", new PathPlannerAuto("1P_SLL-IJ1"));
    m_autoChooser.addOption("1P_SLL-I1", new PathPlannerAuto("1P_SLL-I1"));
    m_autoChooser.addOption("1P_SLL-I4", new PathPlannerAuto("1P_SLL-I4"));
    m_autoChooser.addOption("1P_SLL-J1", new PathPlannerAuto("1P_SLL-J1"));
    m_autoChooser.addOption("1P_SLL-J4", new PathPlannerAuto("1P_SLL-J4"));
    // Starting Line Center (SLC)
    m_autoChooser.addOption("1P_SLC-GH1", new PathPlannerAuto("1P_SLC-GH1"));
    m_autoChooser.addOption("1P_SLC-G1", new PathPlannerAuto("1P_SLC-G1"));
    m_autoChooser.addOption("1P_SLC-G2", new PathPlannerAuto("1P_SLC-G2"));
    m_autoChooser.addOption("1P_SLC-G3", new PathPlannerAuto("1P_SLC-G3"));
    m_autoChooser.addOption("1P_SLC-G4", new PathPlannerAuto("1P_SLC-G4"));
    m_autoChooser.addOption("1P_SLC-H1", new PathPlannerAuto("1P_SLC-H1"));
    m_autoChooser.addOption("1P_SLC-H2", new PathPlannerAuto("1P_SLC-H2"));
    m_autoChooser.addOption("1P_SLC-H3", new PathPlannerAuto("1P_SLC-H3"));
    m_autoChooser.addOption("1P_SLC-H4", new PathPlannerAuto("1P_SLC-H4"));
    // Starting Line Right (SLR)
    m_autoChooser.addOption("1P_SLR-EF1", new PathPlannerAuto("1P_SLR-EF1"));
    m_autoChooser.addOption("1P_SLR-E1", new PathPlannerAuto("1P_SLR-E1"));
    m_autoChooser.addOption("1P_SLR-E4", new PathPlannerAuto("1P_SLR-E4"));
    m_autoChooser.addOption("1P_SLR-F1", new PathPlannerAuto("1P_SLR-F1"));
    m_autoChooser.addOption("1P_SLR-F4", new PathPlannerAuto("1P_SLR-F4"));
    // 2 Piece
    // SLL
    m_autoChooser.addOption("2P_SLL-IJ1-CS1L-L4", new PathPlannerAuto("2P_SLL-IJ1-CS1L-L4"));
    m_autoChooser.addOption("2P_SLL-IJ1-CS1L-K4", new PathPlannerAuto("2P_SLL-IJ1-CS1L-K4"));
    m_autoChooser.addOption("2P_SLL-IJ1-CS1L-A4", new PathPlannerAuto("2P_SLL-IJ1-CS1L-A4"));
    m_autoChooser.addOption("2P_SLL-I1-CS1L-L4", new PathPlannerAuto("2P_SLL-I1-CS1L-L4"));
    m_autoChooser.addOption("2P_SLL-I1-CS1L-K4", new PathPlannerAuto("2P_SLL-I1-CS1L-K4"));
    m_autoChooser.addOption("2P_SLL-I1-CS1L-A4", new PathPlannerAuto("2P_SLL-I1-CS1L-A4"));
    m_autoChooser.addOption("2P_SLL-I4-CS1L-L4", new PathPlannerAuto("2P_SLL-I4-CS1L-L4"));
    m_autoChooser.addOption("2P_SLL-I4-CS1L-K4", new PathPlannerAuto("2P_SLL-I4-CS1L-K4"));
    m_autoChooser.addOption("2P_SLL-I4-CS1L-A4", new PathPlannerAuto("2P_SLL-I4-CS1L-A4"));
    m_autoChooser.addOption("2P_SLL-J1-CS1L-L4", new PathPlannerAuto("2P_SLL-J1-CS1L-L4"));
    m_autoChooser.addOption("2P_SLL-J1-CS1L-K4", new PathPlannerAuto("2P_SLL-J1-CS1L-K4"));
    m_autoChooser.addOption("2P_SLL-J1-CS1L-A4", new PathPlannerAuto("2P_SLL-J1-CS1L-A4"));
    m_autoChooser.addOption("2P_SLL-J4-CS1L-L4", new PathPlannerAuto("2P_SLL-J4-CS1L-L4"));
    m_autoChooser.addOption("2P_SLL-J4-CS1L-K4", new PathPlannerAuto("2P_SLL-J4-CS1L-K4"));
    m_autoChooser.addOption("2P_SLL-J4-CS1L-A4", new PathPlannerAuto("2P_SLL-J4-CS1L-A4"));
    // SLC
    m_autoChooser.addOption("2P_SLC-GH1-CS1L-L4", new PathPlannerAuto("2P_SLC-GH1-CS1L-L4"));
    m_autoChooser.addOption("2P_SLC-GH1-CS1L-K4", new PathPlannerAuto("2P_SLC-GH1-CS1L-K4"));
    m_autoChooser.addOption("2P_SLC-GH1-CS1L-A4", new PathPlannerAuto("2P_SLC-GH1-CS1L-A4"));
    m_autoChooser.addOption("2P_SLC-GH1-CS2R-B4", new PathPlannerAuto("2P_SLC-GH1-CS2R-B4"));
    m_autoChooser.addOption("2P_SLC-GH1-CS2R-C4", new PathPlannerAuto("2P_SLC-GH1-CS2R-C4"));
    m_autoChooser.addOption("2P_SLC-GH1-CS2R-D4", new PathPlannerAuto("2P_SLC-GH1-CS2R-D4"));
    m_autoChooser.addOption("2P_SLC-G1-CS2R-B4", new PathPlannerAuto("2P_SLC-G1-CS2R-B4"));
    m_autoChooser.addOption("2P_SLC-G1-CS2R-C4", new PathPlannerAuto("2P_SLC-G1-CS2R-C4"));
    m_autoChooser.addOption("2P_SLC-G1-CS2R-D4", new PathPlannerAuto("2P_SLC-G1-CS2R-D4"));
    m_autoChooser.addOption("2P_SLC-G4-CS2R-B4", new PathPlannerAuto("2P_SLC-G4-CS2R-B4"));
    m_autoChooser.addOption("2P_SLC-G4-CS2R-C4", new PathPlannerAuto("2P_SLC-G4-CS2R-C4"));
    m_autoChooser.addOption("2P_SLC-G4-CS2R-D4", new PathPlannerAuto("2P_SLC-G4-CS2R-D4"));
    m_autoChooser.addOption("2P_SLC-H1-CS1L-L4", new PathPlannerAuto("2P_SLC-H1-CS1L-L4"));
    m_autoChooser.addOption("2P_SLC-H1-CS1L-K4", new PathPlannerAuto("2P_SLC-H1-CS1L-K4"));
    m_autoChooser.addOption("2P_SLC-H1-CS1L-A4", new PathPlannerAuto("2P_SLC-H1-CS1L-A4"));
    m_autoChooser.addOption("2P_SLC-H4-CS1L-L4", new PathPlannerAuto("2P_SLC-H4-CS1L-L4"));
    m_autoChooser.addOption("2P_SLC-H4-CS1L-K4", new PathPlannerAuto("2P_SLC-H4-CS1L-K4"));
    m_autoChooser.addOption("2P_SLC-H4-CS1L-A4", new PathPlannerAuto("2P_SLC-H4-CS1L-A4"));
    // SLR
    m_autoChooser.addOption("2P_SLR-EF1-CS2R-B4", new PathPlannerAuto("2P_SLR-EF1-CS2R-B4"));
    m_autoChooser.addOption("2P_SLR-EF1-CS2R-C4", new PathPlannerAuto("2P_SLR-EF1-CS2R-C4"));
    m_autoChooser.addOption("2P_SLR-EF1-CS2R-D4", new PathPlannerAuto("2P_SLR-EF1-CS2R-D4"));
    m_autoChooser.addOption("2P_SLR-E1-CS2R-B4", new PathPlannerAuto("2P_SLR-E1-CS2R-B4"));
    m_autoChooser.addOption("2P_SLR-E1-CS2R-C4", new PathPlannerAuto("2P_SLR-E1-CS2R-C4"));
    m_autoChooser.addOption("2P_SLR-E1-CS2R-D4", new PathPlannerAuto("2P_SLR-E1-CS2R-D4"));
    m_autoChooser.addOption("2P_SLR-E4-CS2R-B4", new PathPlannerAuto("2P_SLR-E4-CS2R-B4"));
    m_autoChooser.addOption("2P_SLR-E4-CS2R-C4", new PathPlannerAuto("2P_SLR-E4-CS2R-C4"));
    m_autoChooser.addOption("2P_SLR-E4-CS2R-D4", new PathPlannerAuto("2P_SLR-E4-CS2R-D4"));
    m_autoChooser.addOption("2P_SLR-F1-CS2R-B4", new PathPlannerAuto("2P_SLR-F1-CS2R-B4"));
    m_autoChooser.addOption("2P_SLR-F1-CS2R-C4", new PathPlannerAuto("2P_SLR-F1-CS2R-C4"));
    m_autoChooser.addOption("2P_SLR-F1-CS2R-D4", new PathPlannerAuto("2P_SLR-F1-CS2R-D4"));
    m_autoChooser.addOption("2P_SLR-F4-CS2R-B4", new PathPlannerAuto("2P_SLR-F4-CS2R-B4"));
    m_autoChooser.addOption("2P_SLR-F4-CS2R-C4", new PathPlannerAuto("2P_SLR-F4-CS2R-C4"));
    m_autoChooser.addOption("2P_SLR-F4-CS2R-D4", new PathPlannerAuto("2P_SLR-F4-CS2R-D4"));

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
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(() -> m_driveSubsystem.zeroYaw(), m_driveSubsystem)
                .withName("ZeroYaw"));

    /* Pathfinding */
    // Closest REEF BRANCH
    m_driverController
        .y()
        .onTrue(
            PathfindingCommands.pathfindToClosestBranch(
                    m_driveSubsystem,
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                    m_driverController.y().negate())
                .withName("PathfindToBranch"));
    // Closest CORAL STATION
    m_driverController
        .leftBumper()
        .onTrue(
            PathfindingCommands.pathfindToClosestCoralStation(
                m_driveSubsystem,
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                m_driverController.leftBumper().negate()));

    /* Scoring commands */
    // Score
    m_driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED),
                m_CEESubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_CEESubsystem.setPercentSpeed(0);
                  m_AEESubsystem.setPercentSpeed(0);
                },
                m_AEESubsystem,
                m_CEESubsystem))
        .and(
            m_auxButtonBoard.axisGreaterThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5))
        .onTrue(
            new InstantCommand(
                () -> {
                  m_CEESubsystem.setPercentSpeed(0);
                  m_AEESubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED);
                },
                m_AEESubsystem,
                m_CEESubsystem));
    // Intaking
    m_driverController
        .rightTrigger()
        .onTrue(
            SuperstructureCommands.intakeCoral(
                    m_periscopeSubsystem,
                    m_algaePivotSubsystem,
                    m_AEESubsystem,
                    m_CEESubsystem,
                    m_funnelSubsystem)
                .until(m_driverController.rightTrigger().negate())
                .withName("CoralIntake"))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem));

    /* Misc */
    m_driverController
        .back()
        .onTrue(
            new InstantCommand(() -> m_periscopeSubsystem.resetPosition(0), m_periscopeSubsystem)
                .ignoringDisable(true));
  }

  /** Aux Button Board Controls */
  public void auxButtonBoardBindings() {
    /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    /* Score */
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.SCORE.BUTTON_ID)
        .onTrue(SuperstructureCommands.score(m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

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
                m_funnelSubsystem));
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
                m_funnelSubsystem));
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
        .onTrue(SuperstructureCommands.positionsToNet(m_periscopeSubsystem, m_algaePivotSubsystem));
    // Ground ALGAE
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.GROUND_ALGAE.BUTTON_ID)
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
    // Zero mechanisms
    m_auxButtonBoard
        .axisGreaterThan(OperatorConstants.BUTTON_BOARD.CLIMB_DEPLOY.BUTTON_ID, 0.5)
        .onTrue(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Climb ~~~~~~~~~~~~~~~~~~~~ */
    // Deploy Climber
    m_auxButtonBoard
        .axisGreaterThan(OperatorConstants.BUTTON_BOARD.CLIMB_DEPLOY.BUTTON_ID, 0.5)
        .onTrue(
            new InstantCommand(
                () ->
                    m_climberSubsystem.setVoltage(
                        RobotStateConstants.MAX_VOLTAGE * ClimberConstants.DEPLOY_PERCENT_SPEED),
                m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // // Retract Climber
    m_auxButtonBoard
        .axisLessThan(OperatorConstants.BUTTON_BOARD.CLIMB_RETRACT.BUTTON_ID, -0.5)
        .onTrue(
            new InstantCommand(
                () ->
                    m_climberSubsystem.setVoltage(
                        RobotStateConstants.MAX_VOLTAGE * ClimberConstants.RETRACT_PERCENT_SPEED),
                m_climberSubsystem))
        .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Pathfinding Selection ~~~~~~~~~~~~~~~~~~~~ */
    // REEF Face AB
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_AB.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("A", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToA"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch("B", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToB"));
    // REEF Face CD
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_CD.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("C", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToC"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch("D", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToD"));
    // REEF Face EF
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_EF.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("F", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToF"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "E", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face GH
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_GH.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("H", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToH"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "G", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face IJ
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_IJ.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("J", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToJ"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch("I", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToI"));
    // REEF Face KL
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_KL.BUTTON_ID)
        .and(m_driverController.leftTrigger()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("K", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToK"))
        .and(
            m_auxButtonBoard.axisLessThan(
                OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID,
                -0.5)) // Pathfind to right branch (Driver POV) if switch is toggled
        .onTrue(
            PathfindingCommands.pathfindToBranch("L", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToL"));
  }

  /** Aux Xbox Controls */
  public void auxControllerBindings() {
    // // AEE testing binding
    // m_auxController
    //     .leftTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_AEESubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED),
    //             m_AEESubsystem))
    //     .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));
    // m_auxController
    //     .leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
    //             m_AEESubsystem))
    //     .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));

    // // CEE testing binding
    // m_auxController
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED),
    //             m_CEESubsystem))
    //     .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));
    // m_auxController
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_CEESubsystem.setPercentSpeed(CEEConstants.INTAKE_PERCENT_SPEED),
    //             m_CEESubsystem))
    //     .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));

    // // Funnel testing binding
    // m_auxController
    //     .povLeft()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_funnelSubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED),
    //             m_funnelSubsystem))
    //     .onFalse(new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(0),
    // m_funnelSubsystem));
    // m_auxController
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_funnelSubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED),
    //             m_funnelSubsystem))
    //     .onFalse(new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(0),
    // m_funnelSubsystem));

    // // ALGAE Pivot testing binding
    // m_auxController
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.MAX_ANGLE_RAD),
    //             m_algaePivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //             m_algaePivotSubsystem));
    // m_auxController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.MIN_ANGLE_RAD),
    //             m_algaePivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setAngle(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //             m_algaePivotSubsystem));

    // // Periscope testing binding
    // m_auxController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_periscopeSubsystem.setPosition(Units.inchesToMeters(18)),
    //             m_periscopeSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MIN_HEIGHT_M),
    //             m_periscopeSubsystem));
    // m_auxController
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               m_periscopeSubsystem.enablePID(false);
    //               m_periscopeSubsystem.setVoltage(2);
    //             },
    //             m_periscopeSubsystem))
    //     .onFalse(
    //         new InstantCommand(() -> m_periscopeSubsystem.setVoltage(0), m_periscopeSubsystem));
    // m_auxController
    //     .rightStick()
    //     .onTrue(
    //         new InstantCommand(() -> m_periscopeSubsystem.resetPosition(0),
    // m_periscopeSubsystem));
    // m_auxController
    //     .start()
    //     .onTrue(
    //         new InstantCommand(() -> m_periscopeSubsystem.enablePID(true),
    // m_periscopeSubsystem));
    // m_auxController
    //     .back()
    //     .onTrue(
    //         new InstantCommand(() -> m_periscopeSubsystem.enablePID(false),
    // m_periscopeSubsystem));

    // /* Climb */
    // // Joystick to move
    // m_climberSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () ->
    //             m_climberSubsystem.setVoltage(
    //                 RobotStateConstants.MAX_VOLTAGE * m_auxController.getLeftY()),
    //         m_climberSubsystem));
    // // Deploy
    // m_auxController
    //     .povUp()
    //     .onTrue(new InstantCommand(() -> m_climberSubsystem.setVoltage(2), m_climberSubsystem))
    //     .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // // Retract
    // m_auxController
    //     .povDown()
    //     .onTrue(new InstantCommand(() -> m_climberSubsystem.setVoltage(-2), m_climberSubsystem))
    //     .onFalse(new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    /* Score */
    m_auxController
        .rightTrigger()
        .onTrue(SuperstructureCommands.score(m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

    /* CORAL and ALGAE */
    // L1 or PROCESSOR
    m_auxController
        .a()
        .onTrue(SuperstructureCommands.positionsToL1(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.positionsToProcessor(
                m_periscopeSubsystem, m_algaePivotSubsystem));
    // L2 CORAL or ALGAE
    m_auxController
        .x()
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
        .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.intakeL2Algae(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem));
    // L3 CORAL or ALGAE
    m_auxController
        .b()
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
        .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.intakeL3Algae(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem));
    // L4 or NET
    m_auxController
        .y()
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
        .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
        .onTrue(SuperstructureCommands.positionsToNet(m_periscopeSubsystem, m_algaePivotSubsystem));
    // Ground ALGAE
    m_auxController
        .rightBumper()
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
                m_funnelSubsystem));
    // CORAL Intake
    m_auxController
        .leftTrigger()
        .onTrue(
            SuperstructureCommands.intakeCoral(
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
                m_funnelSubsystem));
    // Zero mechanisms
    m_auxController
        .back()
        .onTrue(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem));
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
    // m_algaePivotSubsystem.enableBrakeMode(enable);
    // m_periscopeSubsystem.enableBrakeMode(enable);
    m_climberSubsystem.enableBrakeMode(enable);
    // m_funnelSubsystem.enableBrakeMode(enable);
    m_AEESubsystem.enableBrakeMode(enable);
    m_CEESubsystem.enableBrakeMode(enable);
  }
}
