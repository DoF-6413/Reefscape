package frc.robot.Subsystems.AlgaeEndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class AEEIOSim implements AEEIO {
  private final FlywheelSim m_sim;

  /**
   * This constructs a new AEEIOSim instance.
   *
   * <p>This creates a new AEEIO object that creates that uses the simulated versions of the NEO
   * motor to run the AEE simulated flywheel
   */
  public AEEIOSim() {
    System.out.println("[Init] Creating AEEIOSim");

    // Initialize flywheel sim with a NEO motor
    m_sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), AEEConstants.MOI_KG_M2, AEEConstants.GEAR_RATIO),
            DCMotor.getNEO(1),
            0);
  }

  @Override
  public void updateInputs(AEEIOInputs inputs) {
    // Update flywheel sim
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
