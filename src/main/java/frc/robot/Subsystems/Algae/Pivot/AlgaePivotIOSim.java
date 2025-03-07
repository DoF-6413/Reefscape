package frc.robot.Subsystems.Algae.Pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class AlgaePivotIOSim implements AlgaePivotIO {
  // Arm system simulation
  private final SingleJointedArmSim m_armSim;
  private double m_voltage = 0.0;

  /**
   * Constructs a new {@link AlgaePivotIOSim} instance.
   *
   * <p>This creates a new {@link AlgaePivotIOSim} object that uses a simulated NEO motor to drive
   * the simulated Pivot (arm) mechanism.
   */
  public AlgaePivotIOSim() {
    System.out.println("[Init] Creating AlgaePivotIOSim");

    // Initialize the simulated Algae Pivot arm with a NEO motor
    m_armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), AlgaePivotConstants.MOI_KG_M2, AlgaePivotConstants.GEAR_RATIO),
            DCMotor.getNEO(1),
            AlgaePivotConstants.GEAR_RATIO,
            AlgaePivotConstants.LENGTH_M,
            AlgaePivotConstants.MIN_ANGLE_RAD,
            AlgaePivotConstants.MAX_ANGLE_RAD,
            AlgaePivotConstants.SIMULATE_GRAVITY,
            AlgaePivotConstants.DEFAULT_ANGLE_RAD); // Starting angle
  }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    // Update arm sim
    m_armSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from the simulated arm system
    inputs.appliedVoltage = m_voltage;
    inputs.currentAmps = Math.abs(m_armSim.getCurrentDrawAmps());
    inputs.absPositionRad= m_armSim.getAngleRads();
    inputs.velocityRadPerSec = m_armSim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_armSim.setInputVoltage(m_voltage);
  }
}
