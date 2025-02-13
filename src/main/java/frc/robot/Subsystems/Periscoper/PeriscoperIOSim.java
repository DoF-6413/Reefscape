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

    inputs.isConnected = new boolean[] {true, true};

    // double voltage = m_PIDController.calculate(inputs.velocityRadPerSec, m_periscoperSP);

    // this.setVoltage(voltage);

    m_periscoperSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.heightMeters = m_periscoperSim.getPositionMeters(); // add a +=
    inputs.velocityRadPerSec =
        m_periscoperSim.getVelocityMetersPerSecond() / PeriscoperConstants.DRUM_RADIUS_M;
    // inputs.appliedVolts = new double[] {voltage, voltage};
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
  public void setVelocity(double velocity) {
    m_periscoperSP = velocity;
  }
  @Override
  public void setPosition(double Pose) { 
    m_PIDController.setSetpoint(Pose);
    m_PIDController.atSetpoint();
  }
}
