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
import frc.robot.Constants.RobotStateConstants;

public class PeriscopeIOTalonFX implements PeriscopeIO {
  // Motor objects
  private final TalonFX[] m_periscopeMotors = new TalonFX[2];
  private final PositionVoltage[] m_motorControllers = new PositionVoltage[2];
  private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

  // Periscope motor inputs
  private StatusSignal<Angle>[] m_positionRot; // Rotations
  private StatusSignal<AngularVelocity>[] m_velocityRotPerSec; // Rotations per second
  private StatusSignal<Voltage>[] m_appliedVolts;
  private StatusSignal<Current>[] m_currentAmps;
  private StatusSignal<Temperature>[] m_tempCelsius;

  public PeriscopeIOTalonFX() {
    System.out.println("[Init] PeriscopeIOTalonFX ");

    m_periscopeMotors[0] = new TalonFX(PeriscopeConstants.CANID_1);
    m_periscopeMotors[1] = new TalonFX(PeriscopeConstants.CANID_2);

    m_motorControllers[0] = new PositionVoltage(0);
    m_motorControllers[1] = new PositionVoltage(0);

    m_motorConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)
        .withControlTimesyncFreqHz(PeriscopeConstants.UPDATE_FREQUENCY_HZ);

    m_motorConfig
        .CurrentLimits
        .withSupplyCurrentLimit(PeriscopeConstants.CUR_LIM_A)
        .withSupplyCurrentLimitEnable(PeriscopeConstants.ENABLE_CUR_LIM)
        .withStatorCurrentLimit(PeriscopeConstants.CUR_LIM_A)
        .withStatorCurrentLimitEnable(PeriscopeConstants.ENABLE_CUR_LIM);

    m_motorConfig
        .Slot0
        .withKP(PeriscopeConstants.KP)
        .withKI(PeriscopeConstants.KI)
        .withKD(PeriscopeConstants.KD)
        .withKS(PeriscopeConstants.KS)
        .withKV(PeriscopeConstants.KV)
        .withKG(PeriscopeConstants.KG);
    m_motorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        1.0 / PeriscopeConstants.UPDATE_FREQUENCY_HZ);

    for (var motor : m_periscopeMotors) {
      motor.setPosition(0.0);
      motor.optimizeBusUtilization();
      motor.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
      motor.getConfigurator().apply(m_motorConfig);
    }

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
    for (int i = 0; i < 2; i++) {
      inputs.isConnected[i] =
          BaseStatusSignal.refreshAll(
                  m_positionRot[i],
                  m_velocityRotPerSec[i],
                  m_appliedVolts[i],
                  m_currentAmps[i],
                  m_tempCelsius[i])
              .isOK();
      inputs.appliedVolts[i] = m_appliedVolts[i].getValueAsDouble();
      inputs.currentDraw[i] = m_currentAmps[i].getValueAsDouble();
      inputs.tempCelsius[i] = m_tempCelsius[i].getValueAsDouble();
    }
    inputs.heightMeters =
        (Units.rotationsToRadians(
                (m_positionRot[0].getValueAsDouble() + m_positionRot[1].getValueAsDouble()) / 2))
            * PeriscopeConstants.GEAR_RATIO
            * (PeriscopeConstants.SPOOL_RADIUS_M * 2 * Math.PI);
    inputs.velocityRadPerSec =
        (Units.rotationsToRadians(
                (m_velocityRotPerSec[0].getValueAsDouble()
                        + m_velocityRotPerSec[1].getValueAsDouble())
                    / 2))
            * PeriscopeConstants.GEAR_RATIO;
  }

  @Override
  public void setVoltage(double volts) {
    for (int i = 0; i < 2; i++) {
      m_periscopeMotors[i].setVoltage(
          MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
    }
  }

  @Override
  public void setPosition(double heightMeters) {
    var positionRotations =
        Units.radiansToRotations(heightMeters / PeriscopeConstants.DRUM_RADIUS_M);
    for (int i = 0; i < 2; i++) {
      m_periscopeMotors[i].setControl(m_motorControllers[i].withPosition(positionRotations));
    }
  }
}
