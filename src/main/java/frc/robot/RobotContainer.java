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
        .whileTrue(
            new InstantCommand(() -> m_driveSubsystem.stopWithX(), m_driveSubsystem)
                .withName("StopWithX"));
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
