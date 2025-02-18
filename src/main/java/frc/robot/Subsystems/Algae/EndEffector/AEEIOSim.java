package frc.robot.Subsystems.Algae.EndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class AEEIOSim implements AEEIO {
  // Flywheel system simulation
  private final FlywheelSim m_flywheelSim;

  /**
   * Constructs a new {@link AEEIOSim} instance.
   *
   * <p>This creates a new {@link AEEIO} object that creates that uses the simulated versions of the
   * NEO motor to run the AEE simulated flywheel.
   */
  public AEEIOSim() {
    System.out.println("[Init] Creating AEEIOSim");

    // Initialize flywheel sim with a NEO motor
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), AEEConstants.MOI_KG_M2, AEEConstants.GEAR_RATIO),
            DCMotor.getNEO(1),
            0);
  }

  @Override
  public void updateInputs(AEEIOInputs inputs) {
    // Update flywheel sim
    m_flywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from the simulated flywheel system
    inputs.velocityRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = m_flywheelSim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_flywheelSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
