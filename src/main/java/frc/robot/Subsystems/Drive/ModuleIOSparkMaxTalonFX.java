// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/** ModuleIO implementation for the real mode of the robot */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {

  // Drive motor
  private final TalonFX m_driveTalonFX;
  private final VelocityVoltage m_driveController = new VelocityVoltage(0);
  private final TalonFXConfiguration m_driveConfig = new TalonFXConfiguration();

  // Turn motor
  private final SparkMax m_turnSparkMax;
  private final SparkMaxConfig m_turnConfig = new SparkMaxConfig();
  // private final SparkClosedLoopController m_turnController;
  private final CANcoder m_turnAbsoluteEncoder;
  private final double m_absoluteEncoderOffsetRad;

  // Drive motor inputs
  private StatusSignal<Angle> m_drivePositionRot; // Rotations
  private StatusSignal<AngularVelocity> m_driveVelocityRotPerSec; // Rotations per second
  private StatusSignal<Voltage> m_driveAppliedVolts;
  private StatusSignal<Current> m_driveCurrentAmps;
  private StatusSignal<Temperature> m_driveTempCelsius;

  // CANcoder inputs
  private StatusSignal<Angle> absoluteEncoderPositionRot;
  private StatusSignal<AngularVelocity> absoluteEncoderVelocityRotPerSec;

  /**
   * Constructs a new ModuleIOSparkMaxTalonFX instance
   *
   * <p>This creates a new ModuleIO object that uses the real KrakenX60 and NEO motors to run the
   * Drive and Turn of the Module
   *
   * @param moduleNumber Number of the module
   */
  public ModuleIOSparkMaxTalonFX(int moduleNumber) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX " + moduleNumber);
    // Initialize Drive motors, Turn motors, Turn encoders and their offsets based on the module
    // number
    switch (moduleNumber) {
      case 0:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, "DriveTrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.CAN_ID, "DriveTrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_RIGHT.OFFSET;
        break;

      case 1:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID, "DriveTrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.CAN_ID, "DriveTrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_LEFT.OFFSET;
        break;

      case 2:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID, "DriveTrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.CAN_ID, "DriveTrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_LEFT.OFFSET;
        break;

      case 3:
        m_driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID, "DriveTrain");
        m_turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        m_turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.CAN_ID, "DriveTrain");
        m_absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_RIGHT.OFFSET;
        break;

      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // TalonFX motor configurations
    m_driveConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);

    // TalonFX current limit configurations
    m_driveConfig
        .CurrentLimits
        .withSupplyCurrentLimit(DriveConstants.CUR_LIM_A)
        .withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(DriveConstants.CUR_LIM_A)
        .withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);

    // TalonFX closed loop configurations
    m_driveConfig
        .Slot0
        .withKP(DriveConstants.DRIVE_KP)
        .withKI(DriveConstants.DRIVE_KI)
        .withKD(DriveConstants.DRIVE_KD)
        .withKS(DriveConstants.DRIVE_KS)
        .withKV(DriveConstants.DRIVE_KV);
    m_driveConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / DriveConstants.UPDATE_FREQUENCY_HZ);
    m_driveController.withUpdateFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);

    // SPARK MAX configurations
    m_turnConfig
        .inverted(DriveConstants.TURN_IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.CUR_LIM_A);

    // SPARK MAX closed loop controller configurations
    // m_turnController = m_turnSparkMax.getClosedLoopController();
    // m_turnConfig
    //     .closedLoop
    //     .pid(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD)
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(-Math.PI, Math.PI);

    // Optimize CAN bus usage, disable all signals beside refreshed signals in code
    m_driveTalonFX.optimizeBusUtilization();
    m_turnAbsoluteEncoder.optimizeBusUtilization();

    // Set CAN timeouts
    m_driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    m_turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000);

    // Initilize encoder positions
    m_driveTalonFX.setPosition(0.0);

    // Apply the CTRE configurations
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);

    // Apply all SPARK MAX configurations: inverted, idleMode, Current Limit
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Drive motor signals
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

    // Initialize Absolute Encoder signals
    absoluteEncoderPositionRot = m_turnAbsoluteEncoder.getAbsolutePosition();
    absoluteEncoderPositionRot.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
    absoluteEncoderVelocityRotPerSec = m_turnAbsoluteEncoder.getVelocity();
    absoluteEncoderVelocityRotPerSec.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update all Drive motor signals and check if they are good
    inputs.driveIsConnected =
        BaseStatusSignal.refreshAll(
                m_driveAppliedVolts,
                m_driveCurrentAmps,
                m_driveTempCelsius,
                m_driveVelocityRotPerSec,
                m_drivePositionRot)
            .isOK();
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePositionRot.getValueAsDouble())
            / DriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveVelocityRotPerSec.getValueAsDouble() * 60)
            / DriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVoltage = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = m_driveTempCelsius.getValueAsDouble();

    // Update all Turn motor and encoder signals and check if they are good
    inputs.absoluteEncoderIsConnected =
        BaseStatusSignal.refreshAll(absoluteEncoderPositionRot, absoluteEncoderVelocityRotPerSec)
            .isOK();
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(absoluteEncoderPositionRot.getValueAsDouble())
                        + m_absoluteEncoderOffsetRad)
                .getRadians());
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(absoluteEncoderVelocityRotPerSec.getValueAsDouble())
            / DriveConstants.STEER_GEAR_RATIO;
    inputs.turnAppliedVoltage = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = m_turnSparkMax.getOutputCurrent();
    inputs.turnTempCelsius = m_turnSparkMax.getMotorTemperature();
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

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveTalonFX.setControl(m_driveController.withVelocity(velocityRadPerSec));
  }

  // @Override
  // public void setTurnPosition(Rotation2d position) {
  //   double setpoint = position.getRadians();
  //   m_turnController.setReference(setpoint, ControlType.kPosition);
  // }

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
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    m_driveConfig.Slot0.withKS(kS).withKV(kV);
    m_driveTalonFX.getConfigurator().apply(m_driveConfig);
  }

  // @Override
  // public void setTurnPID(double kP, double kI, double kD) {
  //   m_turnConfig.closedLoop.pid(kP, kI, kD);
  //   m_turnSparkMax.configure(
  //       m_turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  // }

  // @Override
  // public void updateRelativePosition() {
  //   m_turnRelativeEncoder.setPosition(
  //       Units.rotationsToRadians(absoluteEncoderPositionRot.getValueAsDouble()));
  // }
}
