package frc.robot.Subsystems.Periscope;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotStateConstants;

public class PeriscopeIOTalonFX implements PeriscopeIO {
  // Motor, controller, and configurator
  private final TalonFX m_leadTalonFX;
  private final TalonFX m_followerTalonFX;
  private final PositionVoltage[] m_motorControllers = new PositionVoltage[2];
  private final MotionMagicExpoVoltage[] m_motionMagic = new MotionMagicExpoVoltage[2];
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final DigitalInput[] m_hallEffectSensors = new DigitalInput[2];

  // Periscope motors' logged signals
  private StatusSignal<Voltage>[] m_appliedVolts = new StatusSignal[2];
  private StatusSignal<Current>[] m_currentAmps = new StatusSignal[2];
  private StatusSignal<Temperature>[] m_tempCelsius = new StatusSignal[2];
  private StatusSignal<Angle>[] m_positionRot = new StatusSignal[2]; // Rotations
  private StatusSignal<AngularVelocity>[] m_velocityRotPerSec =
      new StatusSignal[2]; // Rotations per second

  /**
   * This constructs a new {@link PeriscopeIOTalonFX} instance.
   *
   * <p>This creates a new {@link PeriscopeIO} object that uses two real KrakenX60 motors to drive
   * the Periscope (elevator) mechanism.
   */
  public PeriscopeIOTalonFX() {
    System.out.println("[Init] PeriscopeIOTalonFX");

    // Initialize the motors
    m_leadTalonFX = new TalonFX(PeriscopeConstants.CAN_ID_LEFT);
    m_followerTalonFX = new TalonFX(PeriscopeConstants.CAN_ID_RIGHT);
    m_followerTalonFX.setControl(
        new Follower(PeriscopeConstants.CAN_ID_LEFT, PeriscopeConstants.IS_INVERTED_FOLLOWER));

    // Initialize the closed loop motor controllers
    m_motorControllers[0] = new PositionVoltage(0);
    m_motorControllers[1] = new PositionVoltage(0);

    // Initialize motion magic controller
    m_motionMagic[0] = new MotionMagicExpoVoltage(0);
    m_motionMagic[1] = new MotionMagicExpoVoltage(0);

    // Initialize the Hall Effect sensors
    for (int i = 0; i < m_hallEffectSensors.length; i++) {
      m_hallEffectSensors[i] = new DigitalInput(PeriscopeConstants.HALL_EFFECT_SENSORS_PORTS[i]);
    }

    // Motor configuration
    m_motorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(PeriscopeConstants.UPDATE_FREQUENCY_HZ)
        .withInverted(
            PeriscopeConstants.IS_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
    m_leadTalonFX.setPosition(0.0);
    m_leadTalonFX.optimizeBusUtilization();
    m_leadTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    m_followerTalonFX.setPosition(0.0);
    m_followerTalonFX.optimizeBusUtilization();
    m_followerTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
            
    // Current limit configuration
    m_motorConfig
        .CurrentLimits
        .withSupplyCurrentLimit(PeriscopeConstants.CUR_LIM_A)
        .withSupplyCurrentLimitEnable(PeriscopeConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(PeriscopeConstants.CUR_LIM_A)
        .withStatorCurrentLimitEnable(PeriscopeConstants.ENABLE_CUR_LIM);

    // PID and Feedforward gains configuration
    m_motorConfig
        .Slot0
        .withKP(PeriscopeConstants.KP)
        .withKI(PeriscopeConstants.KI)
        .withKD(PeriscopeConstants.KD)
        .withKS(PeriscopeConstants.KS)
        .withKV(PeriscopeConstants.KV)
        .withKG(PeriscopeConstants.KG);

    // Motion Magic (motion profiling) configuration
    m_motorConfig
        .MotionMagic
        .withMotionMagicCruiseVelocity(PeriscopeConstants.MAX_VELOCITY_ROT_PER_SEC)
        .withMotionMagicAcceleration(PeriscopeConstants.IDEAL_ACCELERATION_ROT_PER_SEC2);

    // Closed loop controller configuration
    m_motorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / PeriscopeConstants.UPDATE_FREQUENCY_HZ);

    // Apply configurations
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);

    // Initialize logged signals for both motors
    // Left
    m_positionRot[0] = m_leadTalonFX.getPosition();
    m_positionRot[0].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec[0] = m_leadTalonFX.getVelocity();
    m_velocityRotPerSec[0].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts[0] = m_leadTalonFX.getMotorVoltage();
    m_appliedVolts[0].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps[0] = m_leadTalonFX.getStatorCurrent();
    m_currentAmps[0].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius[0] = m_leadTalonFX.getDeviceTemp();
    m_tempCelsius[0].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    // Right
    m_positionRot[1] = m_followerTalonFX.getPosition();
    m_positionRot[1].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec[1] = m_followerTalonFX.getVelocity();
    m_velocityRotPerSec[1].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts[1] = m_followerTalonFX.getMotorVoltage();
    m_appliedVolts[1].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps[1] = m_followerTalonFX.getStatorCurrent();
    m_currentAmps[1].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius[1] = m_followerTalonFX.getDeviceTemp();
    m_tempCelsius[1].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(PeriscopeIOInputs inputs) {
    // Update logged inputs from each motor
    for (int i = 0; i < 2; i++) {
      // Update motor signals and check if they are recieved
      inputs.isConnected[i] =
          BaseStatusSignal.refreshAll(
                  m_positionRot[i],
                  m_velocityRotPerSec[i],
                  m_appliedVolts[i],
                  m_currentAmps[i],
                  m_tempCelsius[i])
              .isOK();
      // Update logged inputs from the motor
      inputs.appliedVolts[i] = m_appliedVolts[i].getValueAsDouble();
      inputs.currentDraw[i] = m_currentAmps[i].getValueAsDouble();
      inputs.tempCelsius[i] = m_tempCelsius[i].getValueAsDouble();
      inputs.positionRot[i] = m_positionRot[i].getValueAsDouble();
    }
    // Update logged inputs for the entire Periscope
    inputs.heightMeters =
        Units.rotationsToRadians(m_positionRot[0].getValueAsDouble())
            / PeriscopeConstants.GEAR_RATIO
            * PeriscopeConstants.DRUM_RADIUS_M;
    inputs.velocityRadPerSec =
        (Units.rotationsToRadians(m_velocityRotPerSec[0].getValueAsDouble()))
            / PeriscopeConstants.GEAR_RATIO;
    inputs.velocityMetersPerSec = inputs.velocityRadPerSec * PeriscopeConstants.DRUM_RADIUS_M;

    // Update logged inputs for each Hall Effect sensor
    for (int i = 0; i < m_hallEffectSensors.length; i++) {
      inputs.isHallEffectSensorTriggered[i] = !m_hallEffectSensors[i].get();
    }
  }

  @Override
  public void setVoltage(double volts) {
    m_leadTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_motorConfig.MotorOutput.withNeutralMode(
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);
  }

  @Override
  public void resetPosition(double height) {
    double positionRot = Units.radiansToRotations(height / PeriscopeConstants.DRUM_RADIUS_M);
    m_leadTalonFX.setPosition(positionRot);
  }

  /**
   * Sets the position of the Periscope using the motors' closed loop controller built into the
   * TalonFX speed controller.
   *
   * @param heightMeters Position of the Periscope in meters.
   */
  @Override
  public void setPosition(double heightMeters) {
    // Convert linear position (meters) to angular position (radians), then convert units from
    // radians to rotations
    var positionRotations =
        Units.radiansToRotations(heightMeters / PeriscopeConstants.DRUM_RADIUS_M);
    m_leadTalonFX.setControl(m_motorControllers[0].withPosition(positionRotations));
    m_leadTalonFX.setControl(
        m_motionMagic[0].withPosition(positionRotations).withFeedForward(heightMeters));
  }

  /**
   * Sets the PID gains of the Periscope motors' built in closed loop controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  @Override
  public void setPID(double kP, double kI, double kD) {
    // Configure new gains
    m_motorConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
    // Apply configuration for each motor
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);
  }

  /**
   * Sets the Feedforward gains for the Periscope motors' built in closed loop controller.
   *
   * @param kS Static gain value.
   * @param kG Gravity gain value.
   * @param kV Velocity gain value.
   * @param kA Acceleration gain value.
   */
  @Override
  public void setFF(double kS, double kG, double kV, double kA) {
    // Configure new gains
    m_motorConfig.Slot0.withKS(kS).withKG(kG).withKV(kV).withKA(kA);
    // Apply configuration for each motor
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);
  }

  @Override
  public void setMaxAcceleration(double acceleration) {
    m_motorConfig.MotionMagic.withMotionMagicAcceleration(acceleration);
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);
  }
}
