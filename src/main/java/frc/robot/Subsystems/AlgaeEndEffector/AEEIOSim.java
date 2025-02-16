package frc.robot.Subsystems.AlgaeEndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class AEEIOSim implements AEEIO {

  private final FlywheelSim m_AEESim;

  public AEEIOSim() {
    System.out.println("[Init] Creating AEEIOSim");

    m_AEESim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), AEEConstants.MOI_KG_M2, AEEConstants.GEAR_RATIO),
            DCMotor.getNeo550(1),
            AEEConstants.GEAR_RATIO);
  }

  @Override
  public void updateInputs(AEEIOInputs inputs) {

    m_AEESim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.isConnected = true;
    inputs.positionRad =
        m_AEESim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.velocityRadPerSec = m_AEESim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = m_AEESim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_AEESim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_AEESim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
