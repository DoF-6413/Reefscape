package frc.robot.Subsystems.Funnel;

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

public class FunnelIOSparkMax implements FunnelIO {
  // Funnel motor
  private final SparkMax m_sparkmax;
  private final RelativeEncoder m_relativeEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig();

  /**
   * This constructs a new FunnelIOSparkMax instance.
   *
   * <p>This creates a new FunnelIO object that uses the real NEO motor to run the real Funnel
   * mechanism
   */
  public FunnelIOSparkMax() {
    m_sparkmax = new SparkMax(FunnelConstants.CAN_ID, MotorType.kBrushless);

    // SPARK MAX configurations
    m_config
        .inverted(FunnelConstants.IS_INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FunnelConstants.CUR_LIM_A);
    m_sparkmax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // Initialize relative encoder from SPARK MAX
    m_relativeEncoder = m_sparkmax.getEncoder();

    // Apply configuration
    m_sparkmax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.appliedVoltage = m_sparkmax.getAppliedOutput() * m_sparkmax.getBusVoltage();
    inputs.positionRad =
        Units.rotationsToRadians(m_relativeEncoder.getPosition()) / FunnelConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
            / FunnelConstants.GEAR_RATIO;
    inputs.currentAmps = m_sparkmax.getOutputCurrent();
    inputs.tempCelsius = m_sparkmax.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkmax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
