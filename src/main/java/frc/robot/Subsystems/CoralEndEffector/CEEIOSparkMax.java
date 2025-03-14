package frc.robot.Subsystems.CoralEndEffector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotStateConstants;

public class CEEIOSparkMax implements CEEIO {
  // CEE motor, encoder, and configurator
  private final SparkMax m_sparkmax;
  private final RelativeEncoder m_relativeEncoder;
  private final SparkMaxConfig m_config = new SparkMaxConfig();
  private final DigitalInput m_beamBreak;

  /**
   * Constructs a new {@link CEEIOSparkMax} instance.
   *
   * <p>This creates a new {@link CEEIO} object that uses a real NEO 550 motor to run the CEE
   * mechanism.
   */
  public CEEIOSparkMax() {
    System.out.println("[Init] Creating CEEIOSparkMax");

    // Initialize the SPARK MAX with a NEO (brushless) motor
    m_sparkmax = new SparkMax(CEEConstants.CAN_ID, MotorType.kBrushless);

    // SPARK MAX configurations
    m_config
        .inverted(CEEConstants.IS_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CEEConstants.CUR_LIM_A);

    // setCANTimeout arguments in miliseconds so multiply by 1000 to convert sec to milisec
    m_sparkmax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC * 1000);

    // Initialize relative encoder from SPARK MAX
    m_relativeEncoder = m_sparkmax.getEncoder();

    // Apply configurations
    m_sparkmax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Beam Breaks
    m_beamBreak = new DigitalInput(CEEConstants.BEAM_BREAK_DIO_PORT);
  }

  @Override
  public void updateInputs(CEEIOInputs inputs) {
    // Update logged inputs from the motor
    inputs.appliedVoltage = m_sparkmax.getAppliedOutput() * m_sparkmax.getBusVoltage();
    inputs.currentAmps = m_sparkmax.getOutputCurrent();
    inputs.tempCelsius = m_sparkmax.getMotorTemperature();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_relativeEncoder.getVelocity())
            / CEEConstants.GEAR_RATIO;

    // Update logged inputs from the Beam Breaks. 0 = entrance, 1 = exit
    // If sensor is NOT broken, returns true, so invert value to match logged variable
    inputs.isbeamBreaksTriggered = m_beamBreak.get();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_sparkmax.configure(
        m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    m_sparkmax.setVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
