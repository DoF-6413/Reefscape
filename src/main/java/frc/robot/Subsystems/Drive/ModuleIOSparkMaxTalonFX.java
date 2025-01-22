// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  // Turn motor
  private final SparkMax turnSparkMax;
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final CANcoder turnAbsoluteEncoder;
  private final double absoluteEncoderOffsetRad;

  // Drive motor inputs
  private StatusSignal<Angle> drivePositionRot; // Rotations
  private StatusSignal<AngularVelocity> driveVelocityRotPerSec; // Rotations per second
  private StatusSignal<Voltage> driveAppliedVolts;
  private StatusSignal<Current> driveCurrentAmps;
  private StatusSignal<Temperature> driveTempCelsius;

  // CANcoder inputs
  private StatusSignal<Angle> absoluteEncoderPositionRot;
  private StatusSignal<AngularVelocity> absoluteEncoderVelocityRotPerSec;

  public ModuleIOSparkMaxTalonFX(int moduleNumber) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX " + moduleNumber);
    // Initialize Drive motors, Turn motors, Turn encoders and their offsets based on the module
    // number
    switch (moduleNumber) {
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

    // TalonFX motor configurations
    driveConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    driveConfig.MotorOutput.withControlTimesyncFreqHz(DriveConstants.UPDATE_FREQUENCY_HZ);

    // TalonFX current limit configurations
    driveConfig.CurrentLimits.withSupplyCurrentLimit(DriveConstants.CUR_LIM_A);
    driveConfig.CurrentLimits.withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    driveConfig.CurrentLimits.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    driveConfig.CurrentLimits.withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);

    // Optimize CAN bus usage, disable all signals beside refreshed signals in code
    driveTalonFX.optimizeBusUtilization();
    turnAbsoluteEncoder.optimizeBusUtilization();

    // Initializes position to 0
    driveTalonFX.setPosition(0.0);

    // SPARK MAX configurations
    turnConfig.inverted(DriveConstants.TURN_IS_INVERTED);
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(DriveConstants.CUR_LIM_A);

    // SPARK MAX closed loop controller configurations
    turnConfig.closedLoop.pid(
        DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);

    // Set CAN timeouts
    driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // driveTalonFX.resetSignalFrequencies(); // TODO: Test without theses lines
    // turnAbsoluteEncoder.resetSignalFrequencies();

    // Apply the CTRE configurations
    driveTalonFX.getConfigurator().apply(driveConfig);

    // Apply all SPARK MAX configurations: inverted, idleMode, Current Limit
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Drive motor signals
    drivePositionRot = driveTalonFX.getPosition();
    driveVelocityRotPerSec = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveCurrentAmps = driveTalonFX.getStatorCurrent();
    driveTempCelsius = driveTalonFX.getDeviceTemp();

    // Initialize Absolute Encoder signals
    absoluteEncoderPositionRot = turnAbsoluteEncoder.getAbsolutePosition();
    absoluteEncoderVelocityRotPerSec = turnAbsoluteEncoder.getVelocity();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Update all Drive motor signals and check if they are good
    inputs.driveIsConnected =
        BaseStatusSignal.refreshAll(
                driveAppliedVolts,
                driveCurrentAmps,
                driveTempCelsius,
                driveVelocityRotPerSec,
                drivePositionRot)
            .isOK();

    inputs.drivePositionRad = Units.rotationsToRadians(drivePositionRot.getValueAsDouble());
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveVelocityRotPerSec.getValueAsDouble() * 60)
            / DriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVoltage = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
    inputs.driveTempCelsius = driveTempCelsius.getValueAsDouble();

    // Update all Turn motor and encoder signals and check if they are good
    inputs.absoluteEncoderIsConnected =
        BaseStatusSignal.refreshAll(absoluteEncoderPositionRot, absoluteEncoderVelocityRotPerSec)
            .isOK();
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
                Units.rotationsToRadians(absoluteEncoderPositionRot.getValueAsDouble()))
            + absoluteEncoderOffsetRad;
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(absoluteEncoderVelocityRotPerSec.getValueAsDouble() * 60)
            / DriveConstants.STEER_GEAR_RATIO;
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
        turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
