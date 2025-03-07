package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOSim implements ClimberIO {
  // Arm system simulation
  private final SingleJointedArmSim m_armSim;
  private double m_voltage = 0.0;

  /**
   * Constructs a new {@link ClimberIOSim} instance.
   *
   * <p>This creates a new {@link ClimberIO} object that uses a simulated KrakenX60 motor to drive
   * the simulated Climber mechanism.
   */
  public ClimberIOSim() {
    System.out.println("[Init] Creating ClimberIOSim");

    // Initialize the simulated Climber arm with a KrakenX60 motor
    m_armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), ClimberConstants.MOI_KG_M2, ClimberConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.LENGTH_M,
            ClimberConstants.MIN_ANGLE_RAD,
            ClimberConstants.MAX_ANGLE_RAD,
            ClimberConstants.SIMULATE_GRAVITY,
            ClimberConstants.MIN_ANGLE_RAD); // Starting height
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update arm sim
    m_armSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from simulated arm system
    inputs.isConnected = true;
    inputs.appliedVoltage = m_voltage;
    inputs.currentAmps = Math.abs(m_armSim.getCurrentDrawAmps());
    inputs.positionRad = m_armSim.getAngleRads();
    inputs.velocityRadPerSec = m_armSim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_armSim.setInputVoltage(m_voltage);
  }
}
