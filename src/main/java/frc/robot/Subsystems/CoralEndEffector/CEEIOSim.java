package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class CEEIOSim implements CEEIO {

  private final FlywheelSim m_CEESim;
  private final PIDController m_PIDController;
  private double m_setPoint = 0.0; // setpoint

  public CEEIOSim() {
    System.out.println("[Init] Creating CEEIOSim");

    m_CEESim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(1), CEEConstants.MOI_KG_M2, CEEConstants.GEAR_RATIO),
            DCMotor.getNeo550(1),
            0);

    m_PIDController = new PIDController(CEEConstants.KP, CEEConstants.KI, CEEConstants.KD);
  }

  @Override
  public void updateInputs(CEEIOInputs inputs) {

    m_CEESim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.positionRad =
        m_CEESim.getAngularVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.velocityRadPerSec = m_CEESim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = m_CEESim.getInputVoltage();
    inputs.currentAmps = Math.abs(m_CEESim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_CEESim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }
}
