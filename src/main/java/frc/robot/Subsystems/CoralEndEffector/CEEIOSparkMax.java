package frc.robot.Subsystems.CoralEndEffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.RobotStateConstants;

public class CEEIOSparkMax implements CEEIO {
    
    //CEE motor
    private final SparkMax m_sparkMax;
    private final RelativeEncoder m_relativeEncoder;
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public CEEIOSparkMax() {
        m_sparkMax = new SparkMax(CEEConstants.CEE_MOTOR_ID, MotorType.kBrushless);

        m_config
        .inverted(CEEConstants.CEE_IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CEEConstants.CUR_LIM_A);
        m_sparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
        m_sparkMax.configure(
            m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_relativeEncoder = m_sparkMax.getEncoder();
    }

    @Override
    public void updateInputs(CEEIOInputs inputs) {
        inputs.appliedVoltage = m_sparkMax.getAppliedOutput();
        inputs.positionRad = Units.rotationsToRadians(Units.rotationsToRadians(m_relativeEncoder.getPosition()))
        / CEEConstants.GEAR_RATIO;
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
        / CEEConstants.GEAR_RATIO;
        inputs.currentAmps = m_sparkMax.getOutputCurrent();
        inputs.tempCelsius = m_sparkMax.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
    m_sparkMax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
    
  }

}
