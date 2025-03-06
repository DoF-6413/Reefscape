package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
                new VisionIOSim(
                    VisionConstants.CAMERA.FRONT.CAMERA_INDEX, m_driveSubsystem::getCurrentPose2d),
                new VisionIOSim(
                    VisionConstants.CAMERA.BACK.CAMERA_INDEX, m_driveSubsystem::getCurrentPose2d));
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

    /* Autonomous Routines */
    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption("Path Planner", new PathPlannerAuto("test1"));
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

    SmartDashboard.putNumber("Funnel Speed Percent", 0);
    SmartDashboard.putNumber("ClimberVoltage", 0);
    SmartDashboard.putNumber("PSVoltage", 0);
    SmartDashboard.putNumber("PSHeight", 0);
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
    // this.auxButtonBoardBindings();
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
    // Closest REEF BRANCH
    m_driverController
        .leftTrigger()
        .onTrue(
            PathfindingCommands.pathfindToClosestBranch(
                    m_driveSubsystem,
                    () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                    m_driverController.leftTrigger().negate())
                .withName("PathfindToBranch"));

    /* Scoring commands */
    // Score
    m_driverController
        .rightTrigger()
        .onTrue(
            SuperstructureCommands.score(m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem)
                .until(m_driverController.rightTrigger().negate())
                .withName("Score"));
    // Intaking
    m_driverController
        .rightBumper()
        .onTrue(
            SuperstructureCommands.coralIntake(
                    m_periscopeSubsystem,
                    m_algaePivotSubsystem,
                    m_AEESubsystem,
                    m_CEESubsystem,
                    m_funnelSubsystem)
                .until(m_driverController.rightBumper().negate())
                .withName("CoralIntake"));
  }

  /** Aux Button Board Controls */
  public void auxButtonBoardBindings() {
    /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    /* Score */
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.SCORE.BUTTON_ID)
        .onTrue(
            SuperstructureCommands.score(
                m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

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
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5)) // Run ALGAE position if switch is toggled
        .onTrue(
            SuperstructureCommands.positionsToProcessor(
                m_periscopeSubsystem, m_algaePivotSubsystem));
    // L2 CORAL or ALGAE
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.L2.BUTTON_ID)
        .onTrue(
            SuperstructureCommands.positionsToL2Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5)) // Run ALGAE position if switch is toggled
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
            SuperstructureCommands.positionsToL3Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5)) // Run ALGAE position if switch is toggled
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
        .onTrue(SuperstructureCommands.positionsToL4(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.zero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_AEESubsystem,
                m_CEESubsystem,
                m_funnelSubsystem))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_CORAL_ALGAE.BUTTON_ID, 0.5)) // Run ALGAE position if switch is toggled
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
                m_funnelSubsystem));
    // Zero mechanisms
    m_auxButtonBoard
        .axisGreaterThan(OperatorConstants.BUTTON_BOARD.ZERO.BUTTON_ID, 0.5)
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
        .axisGreaterThan(OperatorConstants.BUTTON_BOARD.CLIMB.BUTTON_ID, 0.5)
        .onTrue(new InstantCommand(()-> m_climberSubsystem.setVoltage(2), m_climberSubsystem))
        .onFalse(new InstantCommand(()-> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // // Retract Climber
    // m_auxButtonBoard
    //     .axisGreaterThan(OperatorConstants.BUTTON_BOARD.ZERO.BUTTON_ID, 0.5)
    //     .onTrue(new InstantCommand(()-> m_climberSubsystem.setVoltage(-2), m_climberSubsystem))
    //     .onFalse(new InstantCommand(()-> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    /* ~~~~~~~~~~~~~~~~~~~~ Pathfinding Selection ~~~~~~~~~~~~~~~~~~~~ */
    // REEF Face AB
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_AB.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("A", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToAB"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "B",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face CD
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_CD.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(    PathfindingCommands.pathfindToBranch("C", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToCD"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "D",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face EF
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_EF.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("F", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToEF"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "E",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face GH
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_GH.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("H", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToGH"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "G",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face IJ
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_IJ.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("J", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToIJ"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "I",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
    // REEF Face KL
    m_auxButtonBoard
        .button(OperatorConstants.BUTTON_BOARD.REEF_KL.BUTTON_ID)
        .and(m_driverController.rightBumper()) // Only Pathfind with Driver confirmation
        .onTrue(
            PathfindingCommands.pathfindToBranch("K", PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .withName("PathfindToKL"))
        .and(m_auxButtonBoard.axisGreaterThan(OperatorConstants.BUTTON_BOARD.SWITCH_BRANCH.BUTTON_ID, 0.5)) // Pathfind to right branch (Driver POV) if switch is toggled // TODO: Change axis to correct one and the threshold too
        .onTrue(
            PathfindingCommands.pathfindToBranch(
                "L",
                PathPlannerConstants.DEFAULT_WALL_DISTANCE_M));
  }

  /** Aux Xbox Controls */
  public void auxControllerBindings() {
    // AEE testing binding
    m_auxController
        .leftTrigger()
        .onTrue(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED), m_AEESubsystem))
        .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));
    m_auxController
        .leftBumper()
        .onTrue(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED), m_AEESubsystem))
        .onFalse(new InstantCommand(() -> m_AEESubsystem.setPercentSpeed(0.0), m_AEESubsystem));

    // CEE testing binding
    m_auxController
        .rightTrigger()
        .onTrue(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), m_CEESubsystem))
        .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));
    m_auxController
        .rightBumper()
        .onTrue(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(CEEConstants.INTAKE_PERCENT_SPEED), m_CEESubsystem))
        .onFalse(new InstantCommand(() -> m_CEESubsystem.setPercentSpeed(0.0), m_CEESubsystem));

    // Funnel testing binding
    m_auxController
        .povLeft()
        .onTrue(
            new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(AEEConstants.SCORE_PERCENT_SPEED), m_funnelSubsystem))
        .onFalse(new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(0), m_funnelSubsystem));
    m_auxController
        .povRight()
        .onTrue(
            new InstantCommand(() -> m_funnelSubsystem.setPercentSpeed(AEEConstants.INTAKE_PERCENT_SPEED), m_funnelSubsystem))
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
                () ->
                    m_periscopeSubsystem.setPosition(
                        Units.inchesToMeters(SmartDashboard.getNumber("PSHeight", 0))),
                m_periscopeSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MIN_HEIGHT_M),
                m_periscopeSubsystem));
    m_auxController
        .b()
        .onTrue(
            new InstantCommand(
                () -> m_periscopeSubsystem.setVoltage(SmartDashboard.getNumber("PSVoltage", 0)),
                m_periscopeSubsystem))
        .onFalse(
            new InstantCommand(() -> m_periscopeSubsystem.setVoltage(0), m_periscopeSubsystem));
    m_auxController
        .start()
        .onTrue(
            new InstantCommand(() -> m_periscopeSubsystem.resetPosition(0), m_periscopeSubsystem));

    /* Climb */
    // Joystick to move
    m_climberSubsystem.setDefaultCommand(
        new InstantCommand(
            ()-> m_climberSubsystem.setVoltage(RobotStateConstants.MAX_VOLTAGE * m_auxController.getLeftY()),
            m_climberSubsystem));
    // Deploy
    m_auxController
        .povUp()
        .onTrue(
            new InstantCommand(() -> m_climberSubsystem.setVoltage(2), m_climberSubsystem))
        .onFalse(
            new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));
    // Retract
    m_auxController
        .povDown()
        .onTrue(
            new InstantCommand(() -> m_climberSubsystem.setVoltage(-2), m_climberSubsystem))
        .onFalse(
            new InstantCommand(() -> m_climberSubsystem.setVoltage(0), m_climberSubsystem));

    // /* ~~~~~~~~~~~~~~~~~~~~ Superstructure ~~~~~~~~~~~~~~~~~~~~ */
    // /* Score */
    // m_auxController
    //     .rightTrigger()
    //     .onTrue(
    //         SuperstructureCommands.score(
    //             m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

    // /* CORAL and ALGAE */
    // // L1 or PROCESSOR
    // m_auxController
    //     .a()
    //     .onTrue(SuperstructureCommands.positionsToL1(m_periscopeSubsystem, m_algaePivotSubsystem))
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
    //         SuperstructureCommands.positionsToL2Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
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
    //         SuperstructureCommands.positionsToL3Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
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
    //     .onTrue(SuperstructureCommands.positionsToL4(m_periscopeSubsystem, m_algaePivotSubsystem))
    //     .onFalse(
    //         SuperstructureCommands.zero(
    //             m_periscopeSubsystem,
    //             m_algaePivotSubsystem,
    //             m_AEESubsystem,
    //             m_CEESubsystem,
    //             m_funnelSubsystem))
    //     .and(m_auxController.leftBumper()) // Run ALGAE position if switch is toggled
    //     .onTrue(SuperstructureCommands.positionsToNet(m_periscopeSubsystem, m_algaePivotSubsystem));
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
    //         SuperstructureCommands.coralIntake(
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
    m_algaePivotSubsystem.enableBrakeMode(enable);
    m_periscopeSubsystem.enableBrakeMode(enable);
    m_climberSubsystem.enableBrakeMode(enable);
    m_funnelSubsystem.enableBrakeMode(enable);
    m_AEESubsystem.enableBrakeMode(enable);
    m_CEESubsystem.enableBrakeMode(enable);
  }
}
