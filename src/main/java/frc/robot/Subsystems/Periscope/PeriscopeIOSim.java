package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.RobotStateConstants;

public class PeriscopeIOSim implements PeriscopeIO {

  private final ElevatorSim m_periscopeSim;
  private final PIDController m_PIDController;
  private double m_periscopeSP = 0.0; // setpoint

  public PeriscopeIOSim() {
    System.out.println("[Init] Creating PeriscopeIOSim");

    m_periscopeSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                PeriscopeConstants.MASS_KG,
                PeriscopeConstants.DRUM_RADIUS_M,
                PeriscopeConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(2),
            PeriscopeConstants.MIN_HEIGHT_M,
            PeriscopeConstants.MAX_HEIGHT_M,
            true,
            0.0);

    m_PIDController =
        new PIDController(PeriscopeConstants.KP, PeriscopeConstants.KI, PeriscopeConstants.KD);
  }

  @Override
  public void updateInputs(PeriscopeIOInputs inputs) {

    double voltage = m_PIDController.calculate(inputs.heightMeters, m_periscopeSP);
    this.setVoltage(voltage);

    m_periscopeSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.isConnected = new boolean[] {true, true};
    inputs.heightMeters = m_periscopeSim.getPositionMeters() / PeriscopeConstants.DRUM_RADIUS_M;
    inputs.velocityRadPerSec = m_periscopeSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = new double[] {voltage, voltage};
    inputs.currentDraw =
        new double[] {
          Math.abs(m_periscopeSim.getCurrentDrawAmps()),
          Math.abs(m_periscopeSim.getCurrentDrawAmps())
        };
  }

  @Override
  public void setVoltage(double volts) {
    m_periscopeSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setPosition(double heightMeters) {
    m_periscopeSP = heightMeters;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }
}
