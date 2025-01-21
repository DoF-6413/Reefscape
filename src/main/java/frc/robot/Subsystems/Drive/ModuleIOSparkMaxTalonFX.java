// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotStateConstants;

/** Add your docs here. */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {

  // Drive motor
  private final TalonFX driveTalonFX;
  private final MotorOutputConfigs driveMotorConfigs = new MotorOutputConfigs();

  // Turn motor
  private final SparkMax turnSparkMax;
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final CANcoder turnAbsoluteEncoder;
  private final double absoluteEncoderOffsetRad;

  // Drive motor inputs
  private StatusSignal<Angle> drivePositionRad;
  private StatusSignal<AngularVelocity> driveVelocityRadPerSec;
  private StatusSignal<Voltage> driveAppliedVolts;
  private StatusSignal<Current> driveCurrentAmps;
  private StatusSignal<Temperature> driveTempCelsius;

  // CANcoder inputs
  private StatusSignal<Angle> absoluteEncoderPositionRot;
  private StatusSignal<AngularVelocity> absoluteEncoderVelocityRotPerSec;

  public ModuleIOSparkMaxTalonFX(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFx " + index);
    // Initialize Drive motors, turn motors, turn encoders and their effsets
    switch (index) {
      case 0:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, "DriveTrain");
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.CAN_ID, "DriveTrain");
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_RIGHT.OFFSET;
        break;

      case 1:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID, "DriveTrain");
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.CAN_ID, "DriveTrain");
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_LEFT.OFFSET;
        break;

      case 2:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID, "DriveTrain");
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.CAN_ID, "DriveTrain");
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_LEFT.OFFSET;
        break;

      case 3:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID, "DriveTrain");
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder =
            new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.CAN_ID, "DriveTrain");
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_RIGHT.OFFSET;
        break;

      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // Update Kraken configurations *NOTE: inverted = InvertedValue.CounterClockwise_Positive
    driveMotorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    driveMotorConfigs.withNeutralMode(NeutralModeValue.Brake);
    driveMotorConfigs.withControlTimesyncFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);
    driveTalonFX.optimizeBusUtilization();

    // Update SPARK MAX configurations

    turnConfig.inverted(DriveConstants.TURN_IS_INVERTED);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(DriveConstants.CUR_LIM_A);

    // Optimize CANcoder CAN bus ussage
    turnAbsoluteEncoder.optimizeBusUtilization();

    // Set CAN timeouts
    driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Apply the CTRE configurations
    driveTalonFX.getConfigurator().apply(driveMotorConfigs);

    // Apply all SPARK MAX configurations: inverted, idleMode, Current Limit
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnConfig.closedLoop.pid(DriveConstants.TURN_KP, 0, 0);

    // Apply Current Limit Configurations
    CurrentLimitsConfigs currentLimitsConfig =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    currentLimitsConfig.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    driveTalonFX.getConfigurator().apply(currentLimitsConfig);

    // Initializes position to 0
    driveTalonFX.setPosition(0.0);
    driveTalonFX.resetSignalFrequencies();
    // Craete Drive motor Status Signals

    // Create CANcoder Status Signals
    turnAbsoluteEncoder.resetSignalFrequencies();

    drivePositionRad = driveTalonFX.getPosition();
    driveVelocityRadPerSec = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveCurrentAmps = driveTalonFX.getStatorCurrent();
    driveTempCelsius = driveTalonFX.getDeviceTemp();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // TODO: Make sure to update the inputs

    absoluteEncoderPositionRot = turnAbsoluteEncoder.getAbsolutePosition();
    absoluteEncoderVelocityRotPerSec = turnAbsoluteEncoder.getVelocity();
    // Drive motor inputs
    inputs.driveIsConnected = BaseStatusSignal.isAllGood();
    inputs.drivePositionRad = Units.rotationsToRadians(drivePositionRad.getValueAsDouble());
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveVelocityRadPerSec.getValueAsDouble() * 60);
    inputs.driveAppliedVoltage = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = driveTempCelsius.getValueAsDouble();

    // Turn motor inputs
    inputs.absoluteEncoderIsConnected = BaseStatusSignal.isAllGood();
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
                Units.rotationsToRadians(absoluteEncoderPositionRot.getValueAsDouble()))
            + absoluteEncoderOffsetRad;
    inputs.turnVelocityRadPerSec =
        absoluteEncoderVelocityRotPerSec.getValueAsDouble() * 60 / DriveConstants.STEER_GEAR_RATIO;
    inputs.turnAppliedVoltage = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    inputs.turnTempCelsius = turnSparkMax.getMotorTemperature();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalonFX.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
