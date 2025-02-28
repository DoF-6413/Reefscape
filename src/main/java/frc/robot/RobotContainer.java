package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.Subsystems.Gyro.*;
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
        m_algaePivotSubsystem = new AlgaePivot(new AlgaePivotIOSparkMax());
        m_periscopeSubsystem = new Periscope(new PeriscopeIOTalonFX());
        m_climberSubsystem = new Climber(new ClimberIOTalonFX());
        m_funnelSubsystem = new Funnel(new FunnelIOSparkMax());
        m_AEESubsystem = new AEE(new AEEIOSparkMax() {});
        m_CEESubsystem = new CEE(new CEEIOSparkMax());
        m_visionSubsystem =
            new Vision(
                m_driveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.CAMERA.FRONT.CAMERA_INDEX)
                // new VisionIOPhotonVision(VisionConstants.CAMERA.BACK.CAMERA_INDEX)
                );
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
    // AprilTag currently seen
    m_driverController
        .x()
        .onTrue(
            PathfindingCommands.pathfindToCurrentTag(
                    m_driveSubsystem,
                    m_visionSubsystem,
                    () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                    m_driverController.x().negate())
                .withName("PathfindToAprilTag"));
    // AprilTag 18 - REEF
    m_driverController
        .leftTrigger()
        .onTrue(
            PathfindingCommands.pathfindToAprilTag(
                    () -> 18, () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftTrigger().negate())
                .withName("PathfindToAprilTag18"));
    // AprilTag 17 - REEF
    m_driverController
        .leftBumper()
        .onTrue(
            PathfindingCommands.pathfindToAprilTag(
                    () -> 17, () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.leftBumper().negate())
                .withName("PathfindToAprilTag17"));
    // AprilTag 19 - REEF
    m_driverController
        .rightTrigger()
        .onTrue(
            PathfindingCommands.pathfindToAprilTag(
                    () -> 19, () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M)
                .until(m_driverController.rightTrigger().negate())
                .withName("PathfindToAprilTag19"));
    // Closest REEF BRANCH
    m_driverController
        .rightBumper()
        .onTrue(
            PathfindingCommands.pathfindToClosestBranch(
                    m_driveSubsystem,
                    () -> PathPlannerConstants.DEFAULT_WALL_DISTANCE_M,
                    m_driverController.rightBumper().negate())
                .withName("PathfindToBranch"));
  }

  /** Aux Controls */
  public void auxControllerBindings() {
    // AEE testing binding
    m_AEESubsystem.setDefaultCommand(
        new InstantCommand(
            () ->
                m_AEESubsystem.setVoltage(
                    m_auxController.getLeftTriggerAxis() * RobotStateConstants.MAX_VOLTAGE),
            m_AEESubsystem));
    m_auxController
        .leftBumper()
        .onTrue(
            Commands.run(
                () -> {
                  m_AEESubsystem.enablePID(true);

                  m_AEESubsystem.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(1000));
                },
                m_AEESubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_AEESubsystem.setVelocity(0);
                  m_AEESubsystem.enablePID(false);
                },
                m_AEESubsystem));

    // CEE testing binding
    m_CEESubsystem.setDefaultCommand(
        new InstantCommand(
            () ->
                m_CEESubsystem.setVoltage(
                    m_auxController.getRightTriggerAxis() * RobotStateConstants.MAX_VOLTAGE),
            m_CEESubsystem));
    m_auxController
        .rightBumper()
        .onTrue(
            Commands.run(
                () -> {
                  m_CEESubsystem.enablePID(true);

                  m_CEESubsystem.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(1000));
                },
                m_CEESubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_CEESubsystem.setVelocity(0);
                  m_CEESubsystem.enablePID(false);
                },
                m_CEESubsystem));

    // Funnel testing binding
    m_auxController
        .povUp()
        .onTrue(
            Commands.run(
                () -> {
                  m_funnelSubsystem.enablePID(true);

                  m_funnelSubsystem.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(1000));
                },
                m_funnelSubsystem))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_funnelSubsystem.setVelocity(0);
                  m_funnelSubsystem.enablePID(false);
                },
                m_funnelSubsystem));
    m_auxController
        .povDown()
        .onTrue(new InstantCommand(() -> m_funnelSubsystem.setVoltage(12), m_funnelSubsystem))
        .onFalse(new InstantCommand(() -> m_funnelSubsystem.setVoltage(0), m_funnelSubsystem));

    // ALGAE Pivot testing binding
    // m_auxController
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.MAX_ANGLE_RAD),
    //             m_algaePivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //             m_algaePivotSubsystem));
    // m_auxController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.MIN_ANGLE_RAD),
    //             m_algaePivotSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_algaePivotSubsystem.setSetpoint(AlgaePivotConstants.DEFAULT_ANGLE_RAD),
    //             m_algaePivotSubsystem));

    // Periscope testing binding
    // m_auxController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MAX_HEIGHT_M),
    //             m_periscopeSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_periscopeSubsystem.setPosition(PeriscopeConstants.MIN_HEIGHT_M),
    //             m_periscopeSubsystem));

    // // Climber testing binding
    // m_auxController
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_climberSubsystem.setPosition(ClimberConstants.MIN_ANGLE_RAD),
    //             m_climberSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_climberSubsystem.setPosition(ClimberConstants.MAX_ANGLE_RAD),
    //             m_climberSubsystem));

    m_auxController
        .rightTrigger()
        .onTrue(
            SuperstructureCommands.score(
                m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem));

    m_auxController
        .leftTrigger()
        .onTrue(
            SuperstructureCommands.coralPickup(m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .a()
        .onTrue(SuperstructureCommands.positionToL1(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .b()
        .onTrue(
            SuperstructureCommands.positionToL2Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .y()
        .onTrue(
            SuperstructureCommands.positionToL3Coral(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .x()
        .onTrue(SuperstructureCommands.positionToL4(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    // algae
    m_auxController
        .leftBumper()
        .onTrue(
            SuperstructureCommands.positionToProcessor(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .rightBumper()
        .onTrue(SuperstructureCommands.positionToNet(m_periscopeSubsystem, m_algaePivotSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .rightStick()
        .onTrue(
            SuperstructureCommands.positionToL2Algae(m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    m_auxController
        .leftStick()
        .onTrue(
            SuperstructureCommands.positionToL3Algae(m_periscopeSubsystem, m_algaePivotSubsystem, m_AEESubsystem, m_CEESubsystem, m_funnelSubsystem))
        .onFalse(
            SuperstructureCommands.positionToZero(
                m_periscopeSubsystem,
                m_algaePivotSubsystem,
                m_CEESubsystem,
                m_AEESubsystem,
                m_funnelSubsystem));

    // m_auxController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> SuperstructureCommands.setPositions(m_periscopeSubsystem,
    // m_algaePivotSubsystem),
    //             m_algaePivotSubsystem,
    //             m_periscopeSubsystem));

    // m_auxController
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 SuperstructureCommands.setPositions(m_periscopeSubsystem,
    // m_algaePivotSubsystem),
    //             m_algaePivotSubsystem,
    //             m_periscopeSubsystem));
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
