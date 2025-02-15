package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    private final TalonFX m_climbTalonFX;
    private final VelocityVoltage m_climbController = new VelocityVoltage(0);
    private final TalonFXConfiguration m_climbConfig = new TalonFXConfiguration();

    private StatusSignal<Angle> m_climbPosRot; //Position in rotations
    private StatusSignal<AngularVelocity> m_climbVelocityRotPerSec; //Rotations per second
    private StatusSignal<Voltage> m_climbAppliedVolts;
    private StatusSignal<Current> m_climbCurrentAmps;
    private StatusSignal<Temperature> m_climbTempCelsius;

    public ClimberIOTalonFX() {
        System.out.println("[Init] ClimberIOKraken");
        m_climbTalonFX = new TalonFX(ClimberConstants.CAN_ID, "climber");
        m_climbConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withControlTimesyncFreqHz(ClimberConstants.UPDATE_FREQUENCY_HZ);

        m_climbConfig
            .CurrentLimits
            .withSupplyCurrentLimit(ClimberConstants.CUR_LIM_A)
            .withSupplyCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM)
            .withStatorCurrentLimit(ClimberConstants.CUR_LIM_A)
            .withStatorCurrentLimitEnable(ClimberConstants.ENABLE_CUR_LIM);

        m_climbConfig
            .Slot0
            .withKP(ClimberConstants.KP)
            .withKI(ClimberConstants.KI)
            .withKD(ClimberConstants.KD);

        m_climbConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(1.0 / ClimberConstants.UPDATE_FREQUENCY_HZ);

        m_climbTalonFX.setPosition(0.0);
        m_climbTalonFX.optimizeBusUtilization();
        m_climbTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
        m_climbTalonFX.getConfigurator().apply(m_climbConfig);

        m_climbPosRot = m_climbTalonFX.getPosition();
        m_climbPosRot.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
        m_climbVelocityRotPerSec = m_climbTalonFX.getVelocity();
        m_climbVelocityRotPerSec.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
        m_climbAppliedVolts = m_climbTalonFX.getMotorVoltage();
        m_climbAppliedVolts.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
        m_climbCurrentAmps = m_climbTalonFX.getStatorCurrent();
        m_climbCurrentAmps.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
        m_climbTempCelsius = m_climbTalonFX.getDeviceTemp();
        m_climbTempCelsius.setUpdateFrequency(ClimberConstants.UPDATE_FREQUENCY_HZ);
        
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(m_climbPosRot, m_climbVelocityRotPerSec, m_climbAppliedVolts, m_climbCurrentAmps, m_climbTempCelsius).isOK();
        inputs.appliedVoltage = m_climbAppliedVolts.getValueAsDouble();
        inputs.currentAmps = m_climbCurrentAmps.getValueAsDouble();
        inputs.tempCelsius = m_climbTempCelsius.getValueAsDouble();
        inputs.velocityRadPerSec = Units.rotationsToRadians(m_climbVelocityRotPerSec.getValueAsDouble()) * ClimberConstants.GEAR_RATIO;
    }

    @Override
    public void setVoltage(double volts) {
        m_climbTalonFX.setVoltage(MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        m_climbTalonFX.setControl(m_climbController.withVelocity(velocityRadPerSec));
    }
}
