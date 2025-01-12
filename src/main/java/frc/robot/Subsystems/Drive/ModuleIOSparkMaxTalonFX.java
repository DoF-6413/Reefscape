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
  private final StatusSignal<Angle> drivePositionRad;
  private final StatusSignal<AngularVelocity> driveVelocityRadPerSec;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrentAmps;
  private final StatusSignal<Temperature> driveTempCelsius;
  
  // CANcoder inputs
  private final StatusSignal<Angle> absoluteEncoderPositionRad;
  private final StatusSignal<AngularVelocity> absoluteEncoderVelocityRadPerSec;

  public ModuleIOSparkMaxTalonFX(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFx " + index);
    // Initialize Drive motors, turn motors, turn encoders and their effsets
    switch (index) {
      case 0:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.CAN_ID);
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_RIGHT.OFFSET;
        break;

      case 1:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.CAN_ID);
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.FRONT_LEFT.OFFSET;
        break;

      case 2:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.CAN_ID);
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_LEFT.OFFSET;
        break;

      case 3:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.CAN_ID);
        absoluteEncoderOffsetRad = DriveConstants.ABSOLUTE_ENCODER_OFFSET.BACK_RIGHT.OFFSET;
        break;

      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // Update Kraken configurations *NOTE: inverted = InvertedValue.CounterClockwise_Positive
    driveMotorConfigs.withInverted(InvertedValue.Clockwise_Positive);
    driveMotorConfigs.withNeutralMode(NeutralModeValue.Coast);
    driveMotorConfigs.withControlTimesyncFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);
    driveTalonFX.optimizeBusUtilization();

    // Update SPARK MAX configurations
    turnConfig.inverted(DriveConstants.TURN_IS_INVERTED);
    turnConfig.idleMode(IdleMode.kCoast);
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

    // Apply Current Limit Configurations
    CurrentLimitsConfigs currentLimitsConfig =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    currentLimitsConfig.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    driveTalonFX.getConfigurator().apply(currentLimitsConfig);

    // Initializes position to 0
    driveTalonFX.setPosition(0.0);

    // Craete Drive motor Status Signals
    drivePositionRad = driveTalonFX.getPosition();
    driveVelocityRadPerSec = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveCurrentAmps = driveTalonFX.getStatorCurrent();
    driveTempCelsius = driveTalonFX.getDeviceTemp();

    // Create CANcoder Status Signals
    absoluteEncoderPositionRad = turnAbsoluteEncoder.getAbsolutePosition();
    absoluteEncoderVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Drive motor inputs
    inputs.driveIsConnected = BaseStatusSignal.refreshAll(drivePositionRad, driveVelocityRadPerSec, driveAppliedVolts, driveCurrentAmps, driveTempCelsius).isOK();
    inputs.drivePositionRad = Units.rotationsToRadians(drivePositionRad.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveVelocityRadPerSec.getValueAsDouble() * 60);
    inputs.driveAppliedVoltage = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = driveTempCelsius.getValueAsDouble();

    // Turn motor inputs
    inputs.absoluteEncoderIsConnected = BaseStatusSignal.refreshAll(absoluteEncoderPositionRad, absoluteEncoderVelocityRadPerSec).isOK();
    inputs.turnAbsolutePositionRad =
        (Units.rotationsToRadians(absoluteEncoderPositionRad.getValueAsDouble()) + absoluteEncoderOffsetRad)
            / DriveConstants.GEAR_RATIO;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoderVelocityRadPerSec.getValueAsDouble() * 60)
            / DriveConstants.GEAR_RATIO;
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
    turnConfig.inverted(enable);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
