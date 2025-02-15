package frc.robot.Subsystems.Periscoper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.RobotStateConstants;

public class PeriscoperIOSim implements PeriscoperIO {

  private final ElevatorSim m_periscoperSim;
  private final PIDController m_PIDController;
  private double m_periscoperSP = 0.0; // setpoint

  public PeriscoperIOSim() {
    System.out.println("[Init] Creating PeriscoperIOSim");

    m_periscoperSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                PeriscoperConstants.MASS_KG,
                PeriscoperConstants.DRUM_RADIUS_M,
                PeriscoperConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(2),
            PeriscoperConstants.MIN_HEIGHT_M,
            PeriscoperConstants.MAX_HEIGHT_M,
            true,
            0.0);

    m_PIDController =
        new PIDController(PeriscoperConstants.KP, PeriscoperConstants.KI, PeriscoperConstants.KD);
  }

  @Override
  public void updateInputs(PeriscoperIOInputs inputs) {

    double voltage = m_PIDController.calculate(inputs.heightMeters, m_periscoperSP);
    this.setVoltage(voltage);

    m_periscoperSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.isConnected = new boolean[] {true, true};
    inputs.heightMeters = m_periscoperSim.getPositionMeters() / PeriscoperConstants.DRUM_RADIUS_M;
    ;
    inputs.velocityRadPerSec = m_periscoperSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = new double[] {voltage, voltage};
    inputs.currentDraw =
        new double[] {
          Math.abs(m_periscoperSim.getCurrentDrawAmps()),
          Math.abs(m_periscoperSim.getCurrentDrawAmps())
        };
  }

  @Override
  public void setVoltage(double volts) {
    m_periscoperSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setPosition(double heightMeters) {
    m_periscoperSP = heightMeters;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }
}
