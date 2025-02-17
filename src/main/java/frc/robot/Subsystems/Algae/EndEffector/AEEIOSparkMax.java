package frc.robot.Subsystems.Algae.EndEffector;

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
  private final SparkMax m_sparkmax;
  private final RelativeEncoder m_relativeEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig();

  /**
   * This constructs a new AEEIOSparkMax instance.
   *
   * <p>This creates a new AEEIO object that uses the real NEO motor to run the real AEE mechanism
   */
  public AEEIOSparkMax() {
    m_sparkmax = new SparkMax(AEEConstants.CAN_ID, MotorType.kBrushless);

    // SPARK MAX configurations
    m_config
        .inverted(AEEConstants.IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AEEConstants.CUR_LIM_A);
    m_sparkmax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Initialize relative encoder from SPARK MAX
    m_relativeEncoder = m_sparkmax.getEncoder();

    // Apply configuration
    m_sparkmax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(AEEIOInputs inputs) {
    inputs.appliedVoltage = m_sparkmax.getAppliedOutput() * m_sparkmax.getBusVoltage();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
            / AEEConstants.GEAR_RATIO;
    inputs.currentAmps = m_sparkmax.getOutputCurrent();
    inputs.tempCelsius = m_sparkmax.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkmax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
