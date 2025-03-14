package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class CEEIOSim implements CEEIO {
  // Flywheel simulation system
  private final FlywheelSim m_flywheelSim;

  /**
   * Constructs a new {@link CEEIOSim} instance.
   *
   * <p>This creates a new {@link CEEIO} object that creates that uses the simulated versions of the
   * NEO 550 motor to run the CEE simulated flywheel.
   */
  public CEEIOSim() {
    System.out.println("[Init] Creating CEEIOSim");

    // Initialize the flywheel sim with a NEO 550 motor
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(1), CEEConstants.MOI_KG_M2, CEEConstants.GEAR_RATIO),
            DCMotor.getNeo550(1),
            0);
  }

  @Override
  public void updateInputs(CEEIOInputs inputs) {
    // Update the flywheel sim
    m_flywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from the simulated flywheel system
    inputs.appliedVoltage = m_flywheelSim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_flywheelSim.getCurrentDrawAmps());
    inputs.velocityRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double volts) {
    m_flywheelSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
