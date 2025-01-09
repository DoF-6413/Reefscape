// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;

/** Add your docs here. */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {

  private final TalonFX driveTalonFX;
  private final MotorOutputConfigs driveMotorConfigs = new MotorOutputConfigs();

  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;

  private final double absoluteEncoderOffsetRad;
  private final SparkMax turnSparkMax;
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();

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
    // Update SPARK MAX configurations
    turnConfig.inverted(DriveConstants.TURN_IS_INVERTED);
    turnConfig.idleMode(IdleMode.kCoast);
    turnConfig.smartCurrentLimit(DriveConstants.CUR_LIM_A);

    // Update Kraken configurations *NOTE: inverted = clockwise
    driveMotorConfigs.withInverted(DriveConstants.DRIVE_IS_INVERTED);
    driveMotorConfigs.withNeutralMode(NeutralModeValue.Coast);
    driveMotorConfigs.withControlTimesyncFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);

    // Set CAN timeouts
    driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Set encoders to match SPARK MAX Motor Controllers
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

    // Apply all SPARK MAX configurations: inverted, idleMode, Current Limit
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Apply the CTRE configurations
    driveTalonFX.getConfigurator().apply(driveMotorConfigs);

    // Apply Current Limit Configurations
    CurrentLimitsConfigs currentLimitsConfig =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    currentLimitsConfig.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    currentLimitsConfig.withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    driveTalonFX.getConfigurator().apply(currentLimitsConfig);

    // Initializes position to 0
    driveTalonFX.setPosition(0.0);
  }
  /** update the imputs */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveAppliedVoltage = driveTalonFX.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveTalonFX.getSupplyCurrent().getValueAsDouble();
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveTalonFX.getPosition().getValueAsDouble())
            / DriveConstants.GEAR_RATIO;
    inputs.driveTempCelsius = driveTalonFX.getDeviceTemp().getValueAsDouble();
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveTalonFX.getVelocity().getValueAsDouble())
            / DriveConstants.GEAR_RATIO;

    inputs.turnAbsolutePositionRad =
        Units.rotationsToRadians(turnAbsoluteEncoder.getPosition().getValueAsDouble())
            / DriveConstants.GEAR_RATIO;
    inputs.turnAppliedVoltage = turnAbsoluteEncoder.getSupplyVoltage().getValueAsDouble();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnAbsoluteEncoder.getPosition().getValueAsDouble())
            / DriveConstants.GEAR_RATIO;
    inputs.turnTempCelsius = turnSparkMax.getMotorTemperature();
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / DriveConstants.GEAR_RATIO;
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
  }
}
