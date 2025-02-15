package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class FunnelIOSim implements FunnelIO {

  private final FlywheelSim m_funnelSim;
  private final PIDController m_PIDController;
  private double m_setPoint = 0.0; // setpoint

  public FunnelIOSim() {
    System.out.println("[Init] Creating FunnelIOSim");

    m_funnelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), FunnelConstants.MOI_KG_M2, FunnelConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            FunnelConstants.GEAR_RATIO);

    m_PIDController = new PIDController(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {

    m_funnelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.isConnected = true;
    inputs.positionRad = m_funnelSim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC; 
    inputs.velocityRadPerSec =
        m_funnelSim.getAngularVelocityRadPerSec(); 
    inputs.appliedVoltage = m_funnelSim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_funnelSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_funnelSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
