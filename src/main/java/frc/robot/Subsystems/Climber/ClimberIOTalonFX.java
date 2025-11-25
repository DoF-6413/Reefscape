package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class ClimberIOTalonFX implements ClimberIO {
  // Motor, controller, and configurator
  private final TalonFX m_leadTalonFX;
  private final TalonFX m_followerTalonFX;
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
  private final DigitalInput m_limitSwitch;

  // Climber motor's logged signals
  private StatusSignal<Voltage>[] m_appliedVolts = new StatusSignal[2];
  private StatusSignal<Current>[] m_currentAmps = new StatusSignal[2];
  private StatusSignal<Temperature>[] m_tempCelsius = new StatusSignal[2];
  private StatusSignal<Angle>[] m_positionRot = new StatusSignal[2]; // Rotations
  private StatusSignal<AngularVelocity>[] m_velocityRotPerSec =
      new StatusSignal[2]; // Rotations per second

  /**
   * Constructs a new {@link ClimberIOTalonFX} instance.
   *
   * <p>This creates a new {@link ClimberIO} object that uses a real KrakenX60 motor to drive the
   * Climber mechanism.
   */
  public ClimberIOTalonFX() {
    System.out.println("[Init] ClimberIOTalonFX");

    // Initialize the motors and limit switch
    m_leadTalonFX = new TalonFX(ClimberConstants.LEAD_CAN_ID);
    m_followerTalonFX = new TalonFX(ClimberConstants.FOLLOWER_CAN_ID);
    m_followerTalonFX.setControl(
        new Follower(ClimberConstants.LEAD_CAN_ID, ClimberConstants.INVERT_FOLLOWER));
<<<<<<< HEAD
=======
    m_limitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_PORT);
>>>>>>> b9102cc76935759d1f8003713d68906981a93c13

    // Motor configuration
    m_motorConfig
        .MotorOutput
        .withInverted(
            ClimberConstants.IS_INVERTED
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_leadTalonFX.setPosition(0.0);
    m_leadTalonFX.optimizeBusUtilization();
    m_leadTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    m_followerTalonFX.setPosition(0.0);
    m_followerTalonFX.optimizeBusUtilization();
    m_followerTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Current limit configuration
    m_motorConfig
        .CurrentLimits
        .withSupplyCurrentLimit(ClimberConstants.MAX_CURRENT_A)
        .withSupplyCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(ClimberConstants.STALL_CUR_LIM_A)
        .withStatorCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM);

    // Apply configurations
    m_leadTalonFX.getConfigurator().apply(m_motorConfig);

    // Initialize logged signals
    // Lead
    m_positionRot[0] = m_leadTalonFX.getPosition();
    m_positionRot[0].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec[0] = m_leadTalonFX.getVelocity();
    m_velocityRotPerSec[0].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts[0] = m_leadTalonFX.getMotorVoltage();
    m_appliedVolts[0].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps[0] = m_leadTalonFX.getStatorCurrent();
    m_currentAmps[0].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius[0] = m_leadTalonFX.getDeviceTemp();
    m_tempCelsius[0].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    // Follower
    m_positionRot[1] = m_followerTalonFX.getPosition();
    m_positionRot[1].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_velocityRotPerSec[1] = m_followerTalonFX.getVelocity();
    m_velocityRotPerSec[1].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_appliedVolts[1] = m_followerTalonFX.getMotorVoltage();
    m_appliedVolts[1].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_currentAmps[1] = m_followerTalonFX.getStatorCurrent();
    m_currentAmps[1].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
    m_tempCelsius[1] = m_followerTalonFX.getDeviceTemp();
    m_tempCelsius[1].setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Updated logged inputs from each motor
    for (int i = 0; i < 2; i++) {
      // Update signals and check if they are recieved
      inputs.isConnected[i] =
          BaseStatusSignal.refreshAll(
                  m_positionRot[i],
                  m_velocityRotPerSec[i],
                  m_appliedVolts[i],
                  m_currentAmps[i],
                  m_tempCelsius[i])
              .isOK();
      // Update logged inputs from motor
      inputs.appliedVoltage[i] = m_appliedVolts[i].getValueAsDouble();
      inputs.currentAmps[i] = m_currentAmps[i].getValueAsDouble();
      inputs.tempCelsius[i] = m_tempCelsius[i].getValueAsDouble();
    }
    // Update logged inputs of the Climber mechanism
    inputs.positionRad =
        Units.rotationsToRadians(m_positionRot[0].getValueAsDouble()) / ClimberConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(m_velocityRotPerSec[0].getValueAsDouble())
            / ClimberConstants.GEAR_RATIO;
<<<<<<< HEAD
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_leadTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
=======
    inputs.limitSwitch = !m_limitSwitch.get();
>>>>>>> b9102cc76935759d1f8003713d68906981a93c13
  }

  @Override
  public void setVoltage(double volts) {
    m_leadTalonFX.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_leadTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
