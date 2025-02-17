package frc.robot.Subsystems.Climber;

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
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOTalonFX implements ClimberIO {
  // Motor, controller, and configurator
  private final TalonFX m_talonFX;
  private final PositionVoltage m_controller = new PositionVoltage(0);
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Climber motor signals
  private StatusSignal<Angle> m_positionRot; // Rotations
  private StatusSignal<AngularVelocity> m_velocityRotPerSec; // Rotations per second
  private StatusSignal<Voltage> m_appliedVolts;
  private StatusSignal<Current> m_currentAmps;
  private StatusSignal<Temperature> m_tempCelsius;

  /**
   * This constructs a new {@link ClimberIOTalonFX} instance.
   *
   * <p>This creates a new {@link ClimberIO} object that uses a real KrakenX60 motor to drive the
   * Climber mechanism
   */
  public ClimberIOTalonFX() {
    System.out.println("[Init] ClimberIOTalonFX");

    // Initialize the motor
    m_talonFX = new TalonFX(ClimberConstants.CAN_ID);

    // Motor configuration
    m_motorConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(ClimberConstants.UPDATE_FREQUENCY_HZ);

    // Current limit configuration
    m_motorConfig
        .CurrentLimits
        .withSupplyCurrentLimit(ClimberConstants.CUR_LIM_A)
        .withSupplyCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(ClimberConstants.CUR_LIM_A)
        .withStatorCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM);

    // PID gains configuration
    m_motorConfig
        .Slot0
        .withKP(ClimberConstants.KP)
        .withKI(ClimberConstants.KI)
        .withKD(ClimberConstants.KD);

    // Closed loop controller configuration
    m_motorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / ClimberConstants.UPDATE_FREQUENCY_HZ);

    // Reset position
    m_talonFX.setPosition(0.0);

    // Optimize CAN bus usage, disable all signals aside from those refreshed in code
    m_talonFX.optimizeBusUtilization();

    // Timeout CAN after 500 seconds
    m_talonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Apply configurations
    m_talonFX.getConfigurator().apply(m_motorConfig);

    // Initailize logged signals
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
    // Update logged inputs
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
  public void setVoltage(double volts) {
    m_talonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  /**
   * Sets the position of the Climber using the motor's closed loop controller built into the
   * TalonFX speed controller
   *
   * @param heightMeters Angular position of the Climber in radians
   */
  @Override
  public void setPosition(double positionRad) {
    m_talonFX.setControl(m_controller.withPosition(Units.rotationsToRadians(positionRad)));
  }

  /**
   * Sets the PID gains of the Climber motor's built in closed loop controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  @Override
  public void setPID(double kP, double kI, double kD) {
    m_motorConfig.Slot0.withKP(kP).withKI(kI).withKD(kD);
  }
}
