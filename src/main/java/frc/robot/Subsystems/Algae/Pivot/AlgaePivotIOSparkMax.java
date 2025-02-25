package frc.robot.Subsystems.Algae.Pivot;

import com.revrobotics.AbsoluteEncoder;
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

public class AlgaePivotIOSparkMax implements AlgaePivotIO {
  // Motor, encoder, and configurator
  private final SparkMax m_sparkmax;
  private final AbsoluteEncoder m_absoluteEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig();

  /**
   * Constructs a new {@link AlgaePivotIOSparkMax} instance.
   *
   * <p>This creates a new {@link AlgaePivotIO} object that uses the real NEO motor to run the ALGAE
   * Pivot mechanism.
   */
  public AlgaePivotIOSparkMax() {
    System.out.println("[Init] Creating AlgaePivotSparkMax");

    // Initialize the SPARK MAX with a NEO (brushless) motor
    m_sparkmax = new SparkMax(AlgaePivotConstants.CAN_ID, MotorType.kBrushless);

    // Initialize absolute encoder
    m_absoluteEncoder = m_sparkmax.getAbsoluteEncoder();

    // SPARK MAX configurations

    m_config
        .inverted(AlgaePivotConstants.IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaePivotConstants.CUR_LIM_A);
    // setCANTimeout arguments in miliseconds so multiple by 1000 to convert sec to miliseconds
    m_sparkmax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000);

    // Apply configuration
    m_sparkmax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    // Update logged inputs from the motor
    inputs.appliedVoltage = m_sparkmax.getAppliedOutput() * m_sparkmax.getBusVoltage();
    inputs.positionRad =
        Units.rotationsToRadians(m_absoluteEncoder.getPosition()) / AlgaePivotConstants.GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_absoluteEncoder.getVelocity())
            / AlgaePivotConstants.GEAR_RATIO;
    inputs.currentAmps = m_sparkmax.getOutputCurrent();
    inputs.tempCelsius = m_sparkmax.getMotorTemperature();
    
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    // Update configurator
    m_config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    // Apply configuration
    m_sparkmax.configure(
        m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkmax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
