package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Volts;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Gyro.Gyro;
import frc.robot.Utils.HeadingController;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final Module[] m_modules = new Module[4];
  private final Gyro m_gyro;
  private final HeadingController m_headingController = new HeadingController();
  private Twist2d m_twist = new Twist2d();

  // The swerve drive kinematics
  public SwerveDriveKinematics m_swerveDriveKinematics;

  // Previous yaw angle of the robot
  public Rotation2d m_lastRobotYaw = new Rotation2d();

  // Gets previous module positions
  private double[] m_lastModulePositionsMeters = new double[4];

  // System ID
  private SysIdRoutine m_sysId;

  private Rotation2d headingSetpoint = new Rotation2d(-Math.PI / 2);
  /**
   * Constructs a new Drive subsystem instance.
   *
   * <p>This constructor creates a new Drive object that stores the IO implementation of each Module
   * and the Gyroscope.
   *
   * @param FRModuleIO Front Right ModuleIO implementation of the current robot mode
   * @param FLModuleIO Front Left ModuleIO implementation of the current robot mode
   * @param BLModuleIO Back Left ModuleIO implementation of the current robot mode
   * @param BRModuleIO Back Right ModuleIO implementation of the current robot mode
   * @param gyro Gyro subsystem
   */
  public Drive(
      ModuleIO FRModuleIO,
      ModuleIO FLModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      Gyro gyro) {

    // Initialize the Drive subsystem
    System.out.println("[Init] Creating Drive");

    m_gyro = gyro;
    m_modules[0] = new Module(FRModuleIO, 0);
    m_modules[1] = new Module(FLModuleIO, 1);
    m_modules[2] = new Module(BLModuleIO, 2);
    m_modules[3] = new Module(BRModuleIO, 3);

    // Initialize the swerve drive kinematics
    m_swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());

    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("Drive/SysId State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null,
                this));
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      m_modules[i].periodic();
    }
  }

  /**
   * Sets the entire Drive Train to either brake or coast mode
   *
   * @param enable True for brake, false for coast
   */
  public void setBrakeModeAll(boolean enable) {
    for (var module : m_modules) {
      module.setBrakeMode(enable);
    }
  }

  /**
   * Sets the Velocity of the Swerve Drive through Passing in a ChassisSpeeds (Can be Field Relative
   * OR Robot Orientated)
   *
   * @param speeds the desired chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Convert chassis speeds to Swerve Module States, these will be the setpoints for the drive and
    // turn motors
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        m_swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_S);

    // Record chassis speed and module states (setpoint)
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisStates/Setpoints", discreteSpeeds);

    // The current velocity and position of each module
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      measuredStates[i] = m_modules[i].getState();
    }

    // Record optimized setpoints and measured states
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
  }
  
  /**
   * Run each Swerve Module at a specified speed and angle.
   * 
   * @param setpointStates An array of SwerveModuleStates (Module speed in m/s, and the Module angle). The index of the array corresponds to that Module number
   */
  public void runSwerveModules(SwerveModuleState[] setpointStates) {
    // Runs Modules to Run at Specific Setpoints (Linear and Angular Velocity) that
    // is Quick & Optimized for smoothest movement

    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
      optimizedStates[i] = m_modules[i].getState();
    }

    // Updates setpoint logs
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/Measured", optimizedStates);
  }

  /**
   * @return Swerve kinematics configuration of the robot
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDriveKinematics;
  }

  /**
   * @return The position of each Module, distance travelled and wheel angles
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = m_modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Runs the drivetrain with raw values on a scale
   *
   * @param x velociy in x direction of Entire Swerve Drive
   * @param y velocity in y direction of Entire Swerve Drive
   * @param rot Angular Velocity of Entire Swerve Drive
   */
  public void setRaw(double x, double y, double rot) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, this.getRotation()));
  }

  /**
   * @return A list of SwerveModulePositions containing the change in module position and angle
   */
  public SwerveModulePosition[] getWheelDeltas() {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    /* Wheel Deltas or Wheel Positions */
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (m_modules[i].getPositionMeters()
                  - m_lastModulePositionsMeters[i]), // This calculates the change in angle
              m_modules[i].getAngle()); // Gets individual MODULE rotation
      m_lastModulePositionsMeters[i] = m_modules[i].getPositionMeters();
    }
    return wheelDeltas;
  }

  /**
   * Current heading of the robot. Updates based on the Gyro. If gyro is not connected, uses change
   * in module position instead
   *
   * @return The current angle of the robot
   */
  public Rotation2d getRotation() {
    Rotation2d robotYaw;

    /*
     * Twist2d is a change in distance along an arc
     * // x is the forward distance driven
     * // y is the distance driven to the side
     * // (left positive), and the component is the change in heading.
     */
    if (m_gyro.isConnected()) {
      robotYaw = m_gyro.getYaw();
    } else {
      m_twist =
          m_swerveDriveKinematics.toTwist2d(
              getWheelDeltas()); // Updates Twist Based on MODULE position
      robotYaw =
          m_lastRobotYaw.minus(
              new Rotation2d(m_twist.dtheta)); // Updates rotation 2d based on robot module position
    }
    m_lastRobotYaw = robotYaw;
    return robotYaw;
  }

  /**
   * Locks module orientation at 0 degrees and runs drive motors at specified voltage
   *
   * @param output Voltage
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  /**
   * @param direction Forward or Reverse direction
   * @return A quasistatic test in the specified direction
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0)) // Allows module positions to reset
        .withTimeout(1.0)
        .andThen(m_sysId.quasistatic(direction));
  }

  /**
   * @param direction Forward or Reverse direction
   * @return A dynamic test in the specified direction
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0)) // Allows module positions to reset
        .withTimeout(1.0)
        .andThen(m_sysId.dynamic(direction));
  }

  /**
   * @return Average velocity of drive motors in rotations per second, for FeedForward characterization
   */
  public double getAverageDriveVelocity() {
    double velocity = 0.0;
    for (int i = 0; i < 4; i++) {
      velocity += Units.radiansToRotations(m_modules[i].getVelocityRadPerSec());
    }
    return velocity;
  }

  /** Measures the velocity feedforward constants for the drive motors */
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    double rampRateVoltPerSec = 0.1;
    double startDelay = 2;

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  this.runCharacterization(0.0);
                },
                this)
            .withTimeout(startDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * rampRateVoltPerSec;
                  this.runCharacterization(voltage);
                  velocitySamples.add(this.getAverageDriveVelocity());
                  voltageSamples.add(voltage);
                },
                this)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
    }
}
