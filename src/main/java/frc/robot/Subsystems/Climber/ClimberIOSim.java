package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOSim implements ClimberIO {

  private final SingleJointedArmSim m_climberSim;
  private final PIDController m_PIDController;
  private double m_setPoint = 0.0; // setpoint

  public ClimberIOSim() {
    System.out.println("[Init] Creating ClimberIOSim");

    m_climberSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), ClimberConstants.MOI_KG_M2, ClimberConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.LENGHT_M,
            ClimberConstants.CLIMBER_MIN_ANGLE_RAD,
            ClimberConstants.CLIMBER_MAX_ANGLE_RAD,
            false,
            0);

    m_PIDController =
        new PIDController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    double voltage = m_PIDController.calculate(inputs.positionRad, m_setPoint);
    this.setVoltage(voltage);

    m_climberSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.isConnected = true;
    inputs.positionRad = m_climberSim.getAngleRads();
    inputs.velocityRadPerSec = m_climberSim.getVelocityRadPerSec();
    inputs.appliedVoltage = voltage;
    inputs.currentAmps = Math.abs(m_climberSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    m_climberSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setPosition(double position) {
    m_setPoint = position;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }
}
