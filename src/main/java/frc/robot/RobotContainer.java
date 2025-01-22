package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMaxTalonFX;
import frc.robot.Subsystems.Gyro.Gyro;
import frc.robot.Subsystems.Gyro.GyroIO;
import frc.robot.Subsystems.Gyro.GyroIOPigeon2;
import frc.robot.Utils.PoseEstimator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  // Chassis
  private final Drive m_driveSubsystem;
  private final Gyro m_gyroSubsystem;

  // Utils
  private final PoseEstimator m_PoseEstimator;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  // Autos
  private final LoggedDashboardChooser<Command> autoChooser =
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
    m_PoseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem);

    // Adds an "Auto" tab on ShuffleBoard
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());

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

    /** Driver Controls */
    this.driverControllerBindings();
  }

  // Driver Controls
  private void driverControllerBindings() {
    /* Driving the robot */

    m_driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(m_driveSubsystem, m_gyroSubsystem, controller));

    controller
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
    return autoChooser.get();
  }

  public void mechamismsCoastOnDisable(boolean isDisabled) {
    m_driveSubsystem.setBrakeModeAll(!isDisabled);
  }
}
