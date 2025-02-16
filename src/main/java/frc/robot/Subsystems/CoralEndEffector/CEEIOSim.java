package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class CEEIOSim implements CEEIO {

  private final FlywheelSim m_sim;

  /**
   * This constructs a new CEEIOSim instance.
   *
   * <p>This creates a new CEEIO object that creates that uses the simulated versions of the NEO
   * motor to run the CEE simulated flywheel
   */
  public CEEIOSim() {
    System.out.println("[Init] Creating CEEIOSim");

    // Initialize the flywheel sim with a NEO 550 motor
    m_sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(1), CEEConstants.MOI_KG_M2, CEEConstants.GEAR_RATIO),
            DCMotor.getNeo550(1),
            0);
  }

  @Override
  public void updateInputs(CEEIOInputs inputs) {
    // Update the flywheel sim
    m_sim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update inputs
    inputs.positionRad =
        m_sim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = m_sim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_sim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
