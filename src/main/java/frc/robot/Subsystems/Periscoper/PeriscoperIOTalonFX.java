package frc.robot.Subsystems.Periscoper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotStateConstants;

public class PeriscoperIOTalonFX implements PeriscoperIO {
    // Motor objects
    private final TalonFX[] m_periscoperMotors = new TalonFX[2];
    private final VelocityVoltage[] m_motorControllers = new VelocityVoltage[2];
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

    // Periscoper motor inputs
    private StatusSignal<Angle>[] m_positionRot = new StatusSignal[2]; // Rotations
    private StatusSignal<AngularVelocity>[] m_velocityRotPerSec = new StatusSignal[2]; // Rotations per second
    private StatusSignal<Voltage>[] m_appliedVolts = new StatusSignal[2];
    private StatusSignal<Current>[] m_currentAmps = new StatusSignal[2];
    private StatusSignal<Temperature>[] m_tempCelsius = new StatusSignal[2];

    public PeriscoperIOTalonFX() {
        m_periscoperMotors[0] = new TalonFX(PeriscoperConstants.CANID_1);
        m_periscoperMotors[1] = new TalonFX(PeriscoperConstants.CANID_2);

        m_motorControllers[0] = new VelocityVoltage(0);
        m_motorControllers[1] = new VelocityVoltage(0);

        m_motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withControlTimesyncFreqHz(PeriscoperConstants.UPDATE_FREQUENCY_HZ);

        m_motorConfig
            .CurrentLimits
            .withSupplyCurrentLimit(PeriscoperConstants.CUR_LIM_A)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(PeriscoperConstants.CUR_LIM_A)
            .withStatorCurrentLimitEnable(true);

        m_motorConfig
            .Slot0
            .withKP(PeriscoperConstants.KP)
            .withKI(PeriscoperConstants.KI)
            .withKD(PeriscoperConstants.KD)
            .withKS(PeriscoperConstants.KS)
            .withKV(PeriscoperConstants.KV)
            .withKG(PeriscoperConstants.KG);
        m_motorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(1.0 / PeriscoperConstants.UPDATE_FREQUENCY_HZ);

        for (var motor : m_periscoperMotors) {
            motor.setPosition(0.0);
            motor.optimizeBusUtilization();
            motor.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
            motor.getConfigurator().apply(m_motorConfig);
        }

        for (int i = 0; i < 2; i++) {
            m_positionRot[i] = m_periscoperMotors[i].getPosition();
            m_positionRot[i].setUpdateFrequency(PeriscoperConstants.UPDATE_FREQUENCY_HZ);
            m_velocityRotPerSec[i] = m_periscoperMotors[i].getVelocity();
            m_velocityRotPerSec[i].setUpdateFrequency(PeriscoperConstants.UPDATE_FREQUENCY_HZ);
            m_appliedVolts[i] = m_periscoperMotors[i].getMotorVoltage();
            m_appliedVolts[i].setUpdateFrequency(PeriscoperConstants.UPDATE_FREQUENCY_HZ);
            m_currentAmps[i] = m_periscoperMotors[i].getStatorCurrent();
            m_currentAmps[i].setUpdateFrequency(PeriscoperConstants.UPDATE_FREQUENCY_HZ);
            m_tempCelsius[i] = m_periscoperMotors[i].getDeviceTemp();
            m_tempCelsius[i].setUpdateFrequency(PeriscoperConstants.UPDATE_FREQUENCY_HZ);
        }
    }

    @Override
    public void updateInputs(PeriscoperIOInputs inputs) {
        for (int i = 0; i < 2; i++) {
            inputs.isConnected[i] = BaseStatusSignal.refreshAll(m_positionRot[i], m_velocityRotPerSec[i], m_appliedVolts[i], m_currentAmps[i], m_tempCelsius[i]).isOK();
            inputs.appliedVolts[i] = m_appliedVolts[i].getValueAsDouble();
            inputs.currentDraw[i] = m_currentAmps[i].getValueAsDouble();
            inputs.tempCelsius[i] = m_tempCelsius[i].getValueAsDouble();
        }
        inputs.heightMeters = (Units.rotationsToRadians((m_positionRot[0].getValueAsDouble() + m_positionRot[1].getValueAsDouble()) / 2)) * PeriscoperConstants.GEAR_RATIO * (PeriscoperConstants.SPOOL_RADIUS_M * 2 * Math.PI);
        inputs.velocityRadPerSec = (Units.rotationsToRadians((m_velocityRotPerSec[0].getValueAsDouble() + m_velocityRotPerSec[1].getValueAsDouble()) / 2)) * PeriscoperConstants.GEAR_RATIO;
    }

    
}  
