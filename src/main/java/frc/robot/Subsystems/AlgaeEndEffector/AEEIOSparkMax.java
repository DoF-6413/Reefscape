package frc.robot.Subsystems.AlgaeEndEffector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;

public class AEEIOSparkMax implements AEEIO {

  // AEE motor
  private final SparkMax m_sparkMax;
  private final RelativeEncoder m_relativeEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig();

  public AEEIOSparkMax() {
    m_sparkMax =
        new SparkMax(AEEConstants.AEE_MOTOR_ID, MotorType.kBrushless); // TODO: check motor type

    m_config
        .inverted(AEEConstants.AEE_IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AEEConstants.CUR_LIM_A);
    m_sparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    m_sparkMax.configure(
        m_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_relativeEncoder = m_sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(AEEIOInputs inputs) {
    inputs.appliedVoltage = m_sparkMax.getAppliedOutput();
    inputs.positionRad =
        Units.rotationsToRadians(Units.rotationsToRadians(m_relativeEncoder.getPosition()))
            / AEEConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
            / AEEConstants.GEAR_RATIO;
    inputs.currentAmps = m_sparkMax.getOutputCurrent();
    inputs.tempCelsius = m_sparkMax.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkMax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
