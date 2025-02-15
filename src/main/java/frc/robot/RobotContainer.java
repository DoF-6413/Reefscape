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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.TeleopCommands.DriveCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMaxTalonFX;
import frc.robot.Subsystems.Gyro.Gyro;
import frc.robot.Subsystems.Gyro.GyroIO;
import frc.robot.Subsystems.Gyro.GyroIOPigeon2;
import frc.robot.Utils.PathPlanner;
import frc.robot.Utils.PoseEstimator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;
  private final Gyro m_gyroSubsystem;

  // Utils
  private final PoseEstimator m_poseEstimator;
  private final PathPlanner m_pathPlanner;

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
        m_gyroSubsystem = new Gyro(new GyroIOPigeon2());
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                m_gyroSubsystem);
        break;
        // Sim robot, instantiates physics sim IO implementations
      case SIM:
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                m_gyroSubsystem);
        break;
        // Replayed robot, disables all IO implementations
      default:
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                m_gyroSubsystem);
        break;
    }

    // Utils
    m_poseEstimator = new PoseEstimator(m_driveSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
    // Adds an "Auto" tab on ShuffleBoard

    /** Autonomous Routines */
    m_autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption("Path Planner", new PathPlannerAuto("test1"));
    /* SysId Routines */
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive FeedForward Characterization",
        DriveCommands.feedforwardCharacterization(m_driveSubsystem));

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
    // The front of the robot is the side where the intakes are located
    // A default command always runs unless another command is called

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
            () -> -m_driverController.getRightX()));

    // Field relative
    m_driverController
        .y()
        .onTrue(
            DriveCommands.fieldRelativeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()));

    // Robot relative
    m_driverController
        .b()
        .onTrue(
            DriveCommands.robotRelativeDrive(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()));

    // Lock robot heading to 0 degrees
    m_driverController
        .povUp()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> Rotation2d.fromRadians(0)));
    // Lock robot heading to 90 degrees
    m_driverController
        .povLeft()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> Rotation2d.fromRadians(Math.PI / 2)));
    // Lock robot heading to 180 degrees
    m_driverController
        .povDown()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> Rotation2d.fromRadians(Math.PI)));
    // Lock robot heading to -90 degrees
    m_driverController
        .povRight()
        .onTrue(
            DriveCommands.fieldRelativeDriveAtAngle(
                m_driveSubsystem,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> Rotation2d.fromRadians(-Math.PI / 2)));

    // Reset Gyro heading, making the front side of the robot the new 0 degree angle
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem)
                .withName("ZeroYaw"));
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
   * @param enable - True to set brake mode, False to set coast mode
   */
  public void allMechanismsBrakeMode(boolean enable) {
    m_driveSubsystem.setBrakeModeAll(enable);
  }
}
