package frc.robot.Subsystems.Periscope;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private final TalonFX[] m_periscopeMotors = new TalonFX[2];
  private final PositionVoltage[] m_motorControllers = new PositionVoltage[2];
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final DigitalInput[] m_hallEffectSensors = new DigitalInput[6];


  // Periscope motors' logged signals
  private StatusSignal<Voltage>[] m_appliedVolts;
  private StatusSignal<Current>[] m_currentAmps;
  private StatusSignal<Temperature>[] m_tempCelsius;
  private StatusSignal<Angle>[] m_positionRot; // Rotations
  private StatusSignal<AngularVelocity>[] m_velocityRotPerSec; // Rotations per second

  /**
   * This constructs a new {@link PeriscopeIOTalonFX} instance.
   *
   * <p>This creates a new {@link PeriscopeIO} object that uses two real KrakenX60 motors to drive
   * the Periscope (elevator) mechanism.
   */
  public PeriscopeIOTalonFX() {
    System.out.println("[Init] PeriscopeIOTalonFX");

    // Initialize the motors
    m_periscopeMotors[0] = new TalonFX(PeriscopeConstants.CAN_ID_0);
    m_periscopeMotors[1] = new TalonFX(PeriscopeConstants.CAN_ID_1);

    // Initialize the closed loop motor controllers
    m_motorControllers[0] = new PositionVoltage(0);
    m_motorControllers[1] = new PositionVoltage(0);

    // Initialize the hall effect sensors
    for (int i = 0; i < 6; i++) {
      m_hallEffectSensors[i] = new DigitalInput(PeriscopeConstants.HALL_EFFECT_SENSORS[i]);
    }

    // Motor configuration
    m_motorConfig
        .MotorOutput
        .withInverted(
            PeriscopeConstants.IS_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(PeriscopeConstants.UPDATE_FREQUENCY_HZ);

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

    // Misc and apply configuration
    for (var motor : m_periscopeMotors) {
      // Reset position
      motor.setPosition(0.0);

      // Optimize CAN bus usage, disable all signals aside from those refreshed in code
      motor.optimizeBusUtilization();

      // Timeout CAN after 500 seconds
      motor.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

      // Apply configurations
      motor.getConfigurator().apply(m_motorConfig);
    }

    // Initialize logged signals for both motors
    for (int i = 0; i < 2; i++) {
      m_positionRot[i] = m_periscopeMotors[i].getPosition();
      m_positionRot[i].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
      m_velocityRotPerSec[i] = m_periscopeMotors[i].getVelocity();
      m_velocityRotPerSec[i].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
      m_appliedVolts[i] = m_periscopeMotors[i].getMotorVoltage();
      m_appliedVolts[i].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
      m_currentAmps[i] = m_periscopeMotors[i].getStatorCurrent();
      m_currentAmps[i].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
      m_tempCelsius[i] = m_periscopeMotors[i].getDeviceTemp();
      m_tempCelsius[i].setUpdateFrequency(PeriscopeConstants.UPDATE_FREQUENCY_HZ);
    }
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
    }
    // Update logged inputs for the entire Periscope
    inputs.heightMeters =
        Units.rotationsToRadians(
                (m_positionRot[0].getValueAsDouble() + m_positionRot[1].getValueAsDouble()) / 2)
            / PeriscopeConstants.GEAR_RATIO
            * PeriscopeConstants.DRUM_RADIUS_M;
    inputs.velocityRadPerSec =
        (Units.rotationsToRadians(
                (m_velocityRotPerSec[0].getValueAsDouble()
                        + m_velocityRotPerSec[1].getValueAsDouble())
                    / 2))
            / PeriscopeConstants.GEAR_RATIO;
    inputs.velocityMetersPerSec = inputs.velocityRadPerSec * PeriscopeConstants.DRUM_RADIUS_M;
    for (int i = 0; i < 6; i++) {
      inputs.hallEffectSensors[i] = m_hallEffectSensors[i].get();
    }
  }

  @Override
  public void setVoltage(double volts) {
    for (int i = 0; i < 2; i++) {
      m_periscopeMotors[i].setVoltage(
          MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
    }
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_motorConfig.MotorOutput.withNeutralMode(
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);

    for (var motor : m_periscopeMotors) {
      motor.getConfigurator().apply(m_motorConfig);
    }
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
    for (int i = 0; i < 2; i++) {
      m_periscopeMotors[i].setControl(m_motorControllers[i].withPosition(positionRotations));
    }
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
    for (var motor : m_periscopeMotors) {
      motor.getConfigurator().apply(m_motorConfig);
    }
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
    for (var motor : m_periscopeMotors) {
      motor.getConfigurator().apply(m_motorConfig);
    }
  }
}
