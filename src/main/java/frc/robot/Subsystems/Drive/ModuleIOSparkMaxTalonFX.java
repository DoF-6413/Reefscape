package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotStateConstants;
import java.util.Queue;

/** ModuleIO implementation for the real mode of the robot */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {
  // Drive motor, controller, and configurator
  private final TalonFX m_driveTalonFX;
  private final VelocityVoltage m_driveController = new VelocityVoltage(0);
  private final TalonFXConfiguration m_driveConfig = new TalonFXConfiguration();

  // Turn motor, absolute encoder, controller, and configurator
  private final SparkMax m_turnSparkMax;
  private final SparkMaxConfig m_turnConfig = new SparkMaxConfig();
  private final CANcoder m_turnAbsoluteEncoder;
  private final double m_absoluteEncoderOffsetRad;

  // PhoenixOdometryThread queues
  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_absoluteEncoderPositionQueue;

  // Drive motor's logged signals
  private final StatusSignal<Voltage> m_driveAppliedVolts;
  private final StatusSignal<Current> m_driveCurrentAmps;
  private final StatusSignal<Temperature> m_driveTempCelsius;
  private final StatusSignal<Angle> m_drivePositionRot; // Rotations
  private final StatusSignal<AngularVelocity> m_driveVelocityRotPerSec; // Rotations per second

  // CANcoder's logged signals
  private final StatusSignal<Angle> m_absoluteEncoderPositionRot; // Rotations
  private final StatusSignal<AngularVelocity>
      m_absoluteEncoderVelocityRotPerSec; // Rotations per second

  /**
   * Constructs a new {@link ModuleIOSparkMaxTalonFX} instance.
   *
   * <p>This creates a new {@link ModuleIO} object that uses the real KrakenX60 and NEO motors to
   * run the Drive and Turn of the Module and the CANcoder for the absolute Turn position of the
   * wheels.
   *
   * @param moduleNumber Number to the corresponding Swerve Module that is to be initilized.
   */
  public ModuleIOSparkMaxTalonFX(int moduleNumber) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX " + moduleNumber);

    // Initialize Drive motors, Turn motors, Turn encoders and their offsets based on the Module
    // number
    switch (moduleNumber) {
      case 0:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, "Drivetrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.CAN_ID, "Drivetrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_RIGHT.OFFSET;
        break;

      case 1:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID, "Drivetrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.CAN_ID, "Drivetrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_LEFT.OFFSET;
        break;

      case 2:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID, "Drivetrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.CAN_ID, "Drivetrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_LEFT.OFFSET;
        break;

      case 3:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID, "Drivetrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.CAN_ID, "Drivetrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_RIGHT.OFFSET;
        break;

      default:
        throw new RuntimeException("Invalid Module number for ModuleIOSparkMaxTalonFX");
    }

    // TalonFX motor configurations
    m_driveConfig
        .MotorOutput
        .withInverted(
            DriveConstants.DRIVE_IS_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ);

    // TalonFX current limit configurations
    m_driveConfig
        .CurrentLimits
        .withSupplyCurrentLimit(DriveConstants.CUR_LIM_A)
        .withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(DriveConstants.CUR_LIM_A)
        .withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);

    // TalonFX PID and Feedforward gains configuration
    m_driveConfig
        .Slot0
        .withKP(DriveConstants.DRIVE_KP)
        .withKI(DriveConstants.DRIVE_KI)
        .withKD(DriveConstants.DRIVE_KD)
        .withKS(DriveConstants.DRIVE_KS)
        .withKV(DriveConstants.DRIVE_KV);

    // TalonFX closed loop configurations
    m_driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ);
    m_driveController.withUpdateFreqHz(DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ);

    // Initialize Drive encoder position
    m_driveTalonFX.setPosition(0.0);

    // SPARK MAX configurations
    m_turnConfig
        .inverted(DriveConstants.TURN_IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.CUR_LIM_A);

    // Optimize CAN bus usage, disable all signals besides those refreshed in code
    m_driveTalonFX.optimizeBusUtilization();
    m_turnAbsoluteEncoder.optimizeBusUtilization();

    // Set CAN timeouts
    m_driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    m_turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000);

    // Apply TalonFX configurations
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);

    // Apply SPARK MAX configurations
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize logged Drive motor signals
    m_drivePositionRot = m_driveTalonFX.getPosition();
    m_drivePositionRot.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    m_driveVelocityRotPerSec = m_driveTalonFX.getVelocity();
    m_driveVelocityRotPerSec.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    m_driveAppliedVolts = m_driveTalonFX.getMotorVoltage();
    m_driveAppliedVolts.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    m_driveCurrentAmps = m_driveTalonFX.getStatorCurrent();
    m_driveCurrentAmps.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    m_driveTempCelsius = m_driveTalonFX.getDeviceTemp();
    m_driveTempCelsius.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);

    // Initialize logged Absolute Encoder signals
    m_absoluteEncoderPositionRot = m_turnAbsoluteEncoder.getAbsolutePosition();
    m_absoluteEncoderPositionRot.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    m_absoluteEncoderVelocityRotPerSec = m_turnAbsoluteEncoder.getVelocity();
    m_absoluteEncoderVelocityRotPerSec.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);

    // Initialize PhoenixOdometryThread singals
    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_driveTalonFX.getPosition());
    m_absoluteEncoderPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(m_turnAbsoluteEncoder.getAbsolutePosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Drive motor signals and check if they are recieved
    inputs.driveIsConnected =
        BaseStatusSignal.refreshAll(
                m_driveAppliedVolts,
                m_driveCurrentAmps,
                m_driveTempCelsius,
                m_driveVelocityRotPerSec,
                m_drivePositionRot)
            .isOK();
    // Update Drive motor logged inputs
    inputs.driveAppliedVoltage = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = m_driveTempCelsius.getValueAsDouble();
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePositionRot.getValueAsDouble())
            / DriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveVelocityRotPerSec.getValueAsDouble() * 60)
            / DriveConstants.DRIVE_GEAR_RATIO;

    // Update Absolute Encoder signals and check if they are recieved
    inputs.absoluteEncoderIsConnected =
        BaseStatusSignal.refreshAll(
                m_absoluteEncoderPositionRot, m_absoluteEncoderVelocityRotPerSec)
            .isOK();
    // Update Turn motor and Absolute Encoder logged inputs
    inputs.turnAppliedVoltage = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = m_turnSparkMax.getOutputCurrent();
    inputs.turnTempCelsius = m_turnSparkMax.getMotorTemperature();
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(m_absoluteEncoderPositionRot.getValueAsDouble())
                        + m_absoluteEncoderOffsetRad)
                .getRadians());
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(m_absoluteEncoderVelocityRotPerSec.getValueAsDouble())
            / DriveConstants.STEER_GEAR_RATIO;

    // Update omodometry
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        m_absoluteEncoderPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_absoluteEncoderPositionQueue.clear();
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnSparkMax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  /**
   * Sets the velocity of the Drive motor using the closed loop controller built into the TalonFX
   * speed controller.
   *
   * @param velocityRadPerSec Velocity to set Drive motor to in radians per second.
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveTalonFX.setControl(m_driveController.withVelocity(velocityRadPerSec));
  }

  /**
   * Sets the PID gains for the Drive motor's built in closed loop controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    // Configure new gains
    m_driveConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
    // Apply configuration
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }

  /**
   * Sets the Feedforward gains for the Drive motor's built in closed loop controller.
   *
   * @param kS Static gain value.
   * @param kV Velocity gain value.
   */
  @Override
  public void setDriveFF(double kS, double kV) {
    // Configure new gains
    m_driveConfig.Slot0.withKS(kS).withKV(kV);
    // Apply configuration
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }
}
