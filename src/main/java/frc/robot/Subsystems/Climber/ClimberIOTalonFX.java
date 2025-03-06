package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOTalonFX implements ClimberIO {
  // Motor, controller, and configurator
  private final TalonFX m_talonFX;
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Climber motor's logged signals
  private StatusSignal<Voltage> m_appliedVolts;
  private StatusSignal<Current> m_currentAmps;
  private StatusSignal<Temperature> m_tempCelsius;
  private StatusSignal<Angle> m_positionRot; // Rotations
  private StatusSignal<AngularVelocity> m_velocityRotPerSec; // Rotations per second

  /**
   * Constructs a new {@link ClimberIOTalonFX} instance.
   *
   * <p>This creates a new {@link ClimberIO} object that uses a real KrakenX60 motor to drive the
   * Climber mechanism.
   */
  public ClimberIOTalonFX() {
    System.out.println("[Init] ClimberIOTalonFX");

    // Initialize the motor
    m_talonFX = new TalonFX(ClimberConstants.CAN_ID);

    // Motor configuration
    m_motorConfig
        .MotorOutput
        .withInverted(
            ClimberConstants.IS_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(ClimberConstants.UPDATE_FREQUENCY_HZ);

    // Current limit configuration
    m_motorConfig
        .CurrentLimits
        .withSupplyCurrentLimit(ClimberConstants.MAX_CURRENT_A)
        .withSupplyCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(ClimberConstants.STALL_CUR_LIM_A)
        .withStatorCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM);

    // Reset position
    m_talonFX.setPosition(0.0);

    // Optimize CAN bus usage, disable all signals aside from those refreshed in code
    m_talonFX.optimizeBusUtilization();

    // Timeout CAN after 500 seconds
    m_talonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Apply configurations
    m_talonFX.getConfigurator().apply(m_motorConfig);

    // Initialize logged signals
    m_positionRot = m_talonFX.getPosition();
    m_positionRot.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec = m_talonFX.getVelocity();
    m_velocityRotPerSec.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts = m_talonFX.getMotorVoltage();
    m_appliedVolts.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps = m_talonFX.getStatorCurrent();
    m_currentAmps.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius = m_talonFX.getDeviceTemp();
    m_tempCelsius.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update signals and check if they are recieved
    inputs.isConnected =
        BaseStatusSignal.refreshAll(
                m_positionRot, m_velocityRotPerSec, m_appliedVolts, m_currentAmps, m_tempCelsius)
            .isOK();
    // Update logged inputs from motor
    inputs.appliedVoltage = m_appliedVolts.getValueAsDouble();
    inputs.currentAmps = m_currentAmps.getValueAsDouble();
    inputs.tempCelsius = m_tempCelsius.getValueAsDouble();
    inputs.positionRad =
        Units.rotationsToRadians(m_positionRot.getValueAsDouble()) / ClimberConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(m_velocityRotPerSec.getValueAsDouble())
            / ClimberConstants.GEAR_RATIO;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_talonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setVoltage(double volts) {
    m_talonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
