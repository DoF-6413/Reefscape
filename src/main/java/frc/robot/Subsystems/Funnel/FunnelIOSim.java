package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class FunnelIOSim implements FunnelIO {
  // Flywheel system simulation
  private final FlywheelSim m_sim;

  /**
   * Constructs a new {@link FunnelIOSim} instance.
   *
   * <p>This creates a new {@link FunnelIO} object that uses a simulated version of the NEO motor to
   * run the Funnel simulated flywheel
   */
  public FunnelIOSim() {
    System.out.println("[Init] Creating FunnelIOSim");

    // Initialize the flywheel sim with a NEO motor
    m_sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), FunnelConstants.MOI_KG_M2, FunnelConstants.GEAR_RATIO),
            DCMotor.getNEO(1),
            0);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    // Update the flywheel sim
    m_sim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs
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
