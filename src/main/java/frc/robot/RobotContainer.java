package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopCommands.DriveCommands;
import frc.robot.Commands.TeleopCommands.PathfindingCommands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOPigeon2;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMaxTalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;

  // Mechanisms
  //   private final AlgaePivot m_algaePivotSubsystem;
  //   private final Periscope m_periscopeSubsystem;
  //   private final Climber m_climberSubsystem;
  //   private final Funnel m_funnelSubsystem;
  //   private final AEE m_AEESubsystem;
  //   private final CEE m_CEESubsystem;

  // Utils
  //   private final Vision m_visionSubsystem;

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

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
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSparkMax());
        // m_periscopeSubsystem = new Periscope(new PeriscopeIOTalonFX());
        // m_climberSubsystem = new Climber(new ClimberIOTalonFX());
        // m_funnelSubsystem = new Funnel(new FunnelIOSparkMax());
        // m_AEESubsystem = new AEE(new AEEIOSparkMax() {});
        // m_CEESubsystem = new CEE(new CEEIOSparkMax());
        // m_visionSubsystem =
        //     new Vision(
        //         m_driveSubsystem::addVisionMeasurement,
        //         new VisionIOPhotonVision(VisionConstants.CAMERA.FRONT.CAMERA_INDEX)
        //         // new VisionIOPhotonVision(VisionConstants.CAMERA.BACK.CAMERA_INDEX)
        //         );
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
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSim());
        // m_periscopeSubsystem = new Periscope(new PeriscopeIOSim());
        // m_climberSubsystem = new Climber(new ClimberIOSim());
        // m_funnelSubsystem = new Funnel(new FunnelIOSim());
        // m_AEESubsystem = new AEE(new AEEIOSim() {});
        // m_CEESubsystem = new CEE(new CEEIOSim());
        // m_visionSubsystem =
        //     new Vision(
        //         m_driveSubsystem::addVisionMeasurement,
        //         new VisionIOSim(
        //             VisionConstants.CAMERA.FRONT.CAMERA_INDEX,
        // m_driveSubsystem::getCurrentPose2d),
        //         new VisionIOSim(
        //             VisionConstants.CAMERA.BACK.CAMERA_INDEX,
        // m_driveSubsystem::getCurrentPose2d));
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
        // m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIO() {});
        // m_periscopeSubsystem = new Periscope(new PeriscopeIO() {});
        // m_climberSubsystem = new Climber(new ClimberIO() {});
        // m_funnelSubsystem = new Funnel(new FunnelIO() {});
        // m_AEESubsystem = new AEE(new AEEIO() {});
        // m_CEESubsystem = new CEE(new CEEIO() {});
        // m_visionSubsystem = new Vision(m_driveSubsystem::addVisionMeasurement, new VisionIO()
        // {});
        break;
    }

    /* Autonomous Routines */
    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // 1 Piece
    // Starting Line Left (SLL)
    m_autoChooser.addOption("1P_SLL-IJ1", new PathPlannerAuto("1P_SLL-IJ1"));
    m_autoChooser.addOption("1P_SLL-I1", new PathPlannerAuto("1P_SLL-I1"));
    m_autoChooser.addOption("1P_SLL-I4", new PathPlannerAuto("1P_SLL-I4"));
    m_autoChooser.addOption("1P_SLL-J1", new PathPlannerAuto("1P_SLL-J1"));
    m_autoChooser.addOption("1P_SLL-J4", new PathPlannerAuto("1P_SLL-J4"));
    // Starting Line Center (SLC)
    m_autoChooser.addOption("1P_SLC-GH1", new PathPlannerAuto("1P_SLC-GH1"));
    m_autoChooser.addOption("1P_SLC-G!", new PathPlannerAuto("1P_SLC-G1"));
    m_autoChooser.addOption("1P_SLC-G4", new PathPlannerAuto("1P_SLC-G4"));
    m_autoChooser.addOption("1P_SLC-H1", new PathPlannerAuto("1P_SLC-H1"));
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

    /* Test Routines */
    m_autoChooser.addOption("Forward", new PathPlannerAuto("Forward"));
    m_autoChooser.addOption("Forward 180", new PathPlannerAuto("Forward 180"));
    m_autoChooser.addOption("Reverse", new PathPlannerAuto("Reverse"));
    m_autoChooser.addOption("Reverse 180", new PathPlannerAuto("Reverse 180"));
    m_autoChooser.addOption("Diagonal", new PathPlannerAuto("Diagonal"));
    m_autoChooser.addOption("Diagonal 180", new PathPlannerAuto("Diagonal 180"));
    m_autoChooser.addOption("Curve", new PathPlannerAuto("Curve"));
    m_autoChooser.addOption("Curve 180", new PathPlannerAuto("Curve 180"));
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

    Logger.recordOutput("CSPOSES/CS1C", FieldConstants.CORAL_STATION_POSES.get("CS1C"));
    Logger.recordOutput("CSPOSES/CS1L", FieldConstants.CORAL_STATION_POSES.get("CS1L"));
    Logger.recordOutput("CSPOSES/CS1R", FieldConstants.CORAL_STATION_POSES.get("CS1R"));
    Logger.recordOutput("CSPOSES/CS2C", FieldConstants.CORAL_STATION_POSES.get("CS2C"));
    Logger.recordOutput("CSPOSES/CS2L", FieldConstants.CORAL_STATION_POSES.get("CS2L"));
    Logger.recordOutput("CSPOSES/CS2R", FieldConstants.CORAL_STATION_POSES.get("CS2R"));
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
                () -> -m_driverController.getRightX())
            .withName("FieldRelativeDrive"));
    // Field relative
    m_driverController
        .y()
        .onTrue(
            DriveCommands.fieldRelativeDrive(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> -m_driverController.getRightX())
                .withName("FieldRelativeDrive"));
    // Robot relative
    m_driverController
        .b()
        .onTrue(
            DriveCommands.robotRelativeDrive(
                    m_driveSubsystem,
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(),
                    () -> -m_driverController.getRightX())
                .withName("RobotRelativeDrive"));
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

    /* Gyro */
    // Reset Gyro heading, making the front side of the robot the new 0 degree angle
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(() -> m_driveSubsystem.zeroYaw(), m_driveSubsystem)
                .withName("ZeroYaw"));

    /* Pathfinding */
    // AprilTag 18 - REEF
    m_driverController
        .leftTrigger()
        .onTrue(
            PathfindingCommands.pathfindToFieldElement(
                    FieldConstants.getAprilTagPose(18).get().toPose2d(),
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToAprilTag18"));
    // BRANCH A - REEF
    m_driverController
        .leftBumper()
        .onTrue(
            PathfindingCommands.pathfindToFieldElement(
                    FieldConstants.BRANCH_POSES.get("A"),
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M
                        + FieldConstants.BRANCH_TO_WALL_X_M)
                .until(m_driverController.leftBumper().negate())
                .withName("PathfindToBRANCH_A"));
    // CORAL STATION (right, aka 2) Center
    m_driverController
        .rightTrigger()
        .onTrue(
            PathfindingCommands.pathfindToFieldElement(
                    FieldConstants.CORAL_STATION_POSES.get("CS2C"),
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M / 2)
                .until(m_driverController.rightTrigger().negate())
                .withName("PathfindToCS2C"));
    // Closest REEF BRANCH
    m_driverController
        .rightBumper()
        .onTrue(
            PathfindingCommands.pathfindToClosestBranch(
                    m_driveSubsystem,
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                    m_driverController.rightBumper().negate())
                .withName("PathfindToBranch"));
    // Closest CORAL STATION
    m_driverController
        .x()
        .onTrue(
            PathfindingCommands.pathfindToClosestCoralStation(
                    m_driveSubsystem,
                    PathPlannerConstants.DEFAULT_WALL_DISTANCE_M / 2,
                    m_driverController.x().negate())
                .withName("PathfindCS"));
  }

  /** Aux Controls */
  public void auxControllerBindings() {
    //     // AEE testing binding
    //     m_AEESubsystem.setDefaultCommand(
    //         new InstantCommand(
    //             () ->
    //                 m_AEESubsystem.setVoltage(
    //                     m_auxController.getLeftTriggerAxis() * RobotStateConstants.MAX_VOLTAGE),
    //             m_AEESubsystem));
    //     m_auxController
    //         .leftBumper()
    //         .onTrue(
    //             Commands.run(
    //                 () -> {
    //                   m_AEESubsystem.enablePID(true);

    //
    // m_AEESubsystem.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(1000));
    //                 },
    //                 m_AEESubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () -> {
    //                   m_AEESubsystem.setSetpoint(0);
    //                   m_AEESubsystem.enablePID(false);
    //                 },
    //                 m_AEESubsystem));

    //     // CEE testing binding
    //     m_CEESubsystem.setDefaultCommand(
    //         new InstantCommand(
    //             () ->
    //                 m_CEESubsystem.setVoltage(
    //                     m_auxController.getRightTriggerAxis() * RobotStateConstants.MAX_VOLTAGE),
    //             m_CEESubsystem));
    //     m_auxController
    //         .rightBumper()
    //         .onTrue(
    //             Commands.run(
    //                 () -> {
    //                   m_CEESubsystem.enablePID(true);

    //
    // m_CEESubsystem.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(1000));
    //                 },
    //                 m_CEESubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () -> {
    //                   m_CEESubsystem.setSetpoint(0);
    //                   m_CEESubsystem.enablePID(false);
    //                 },
    //                 m_CEESubsystem));

    //     // Funnel testing binding
    //     m_auxController
    //         .povUp()
    //         .onTrue(
    //             Commands.run(
    //                 () -> {
    //                   m_funnelSubsystem.enablePID(true);

    //
    // m_funnelSubsystem.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(1000));
    //                 },
    //                 m_funnelSubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () -> {
    //                   m_funnelSubsystem.setSetpoint(0);
    //                   m_funnelSubsystem.enablePID(false);
    //                 },
    //                 m_funnelSubsystem));
    //     m_auxController
    //         .povDown()
    //         .onTrue(new InstantCommand(() -> m_funnelSubsystem.setVoltage(12),
    // m_funnelSubsystem))
    //         .onFalse(new InstantCommand(() -> m_funnelSubsystem.setVoltage(0),
    // m_funnelSubsystem));

    //     // ALGAE Pivot testing binding
    //     m_auxController
    //         .b()
    //         .onTrue(
    //             new InstantCommand(
    //                 () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.MAX_ANGLE_RAD),
    //                 m_algaePivotSubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () ->
    // m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //                 m_algaePivotSubsystem));
    //     m_auxController
    //         .x()
    //         .onTrue(
    //             new InstantCommand(
    //                 () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.MIN_ANGLE_RAD),
    //                 m_algaePivotSubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () ->
    // m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //                 m_algaePivotSubsystem));

    //     // Periscope testing binding
    //     m_auxController
    //         .a()
    //         .onTrue(
    //             new InstantCommand(
    //                 () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MAX_HEIGHT_M),
    //                 m_periscopeSubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MIN_HEIGHT_M),
    //                 m_periscopeSubsystem));

    //     // Climber testing binding
    //     m_auxController
    //         .y()
    //         .onTrue(
    //             new InstantCommand(
    //                 () -> m_climberSubsystem.setPosition(ClimberConstants.MIN_ANGLE_RAD),
    //                 m_climberSubsystem))
    //         .onFalse(
    //             new InstantCommand(
    //                 () -> m_climberSubsystem.setPosition(ClimberConstants.MAX_ANGLE_RAD),
    //                 m_climberSubsystem));
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
    // m_climberSubsystem.enableBrakeMode(enable);
    // m_funnelSubsystem.enableBrakeMode(enable);
    // m_AEESubsystem.enableBrakeMode(enable);
    // m_CEESubsystem.enableBrakeMode(enable);
  }
}
