package frc.robot.Subsystems.Periscope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.RobotStateConstants;

public class PeriscopeIOSim implements PeriscopeIO {
  // Elevator system simulation
  private final ElevatorSim m_elevatorSim;
  private double m_voltage = 0.0;

  /**
   * This constructs a new {@link PeriscopeIOSim} instance.
   *
   * <p>This creates a new {@link PeriscopeIO} object that uses two simulated KrakenX60 motors to
   * drive the simulated Periscope (elevator) mechanism.
   */
  public PeriscopeIOSim() {
    System.out.println("[Init] Creating PeriscopeIOSim");

    // Initialize elevator simulation with two KarkenX60 motors
    m_elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2),
                PeriscopeConstants.MASS_KG,
                PeriscopeConstants.DRUM_RADIUS_M,
                PeriscopeConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(2),
            PeriscopeConstants.MIN_HEIGHT_M,
            PeriscopeConstants.MAX_HEIGHT_M,
            PeriscopeConstants.SIMULATE_GRAVITY,
            PeriscopeConstants.MIN_HEIGHT_M);
  }

  @Override
  public void updateInputs(PeriscopeIOInputs inputs) {
    // Update elevator sim
    m_elevatorSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from simulated elevator system
    inputs.isConnected = new boolean[] {true, true};
    inputs.appliedVolts = new double[] {m_voltage, m_voltage};
    inputs.currentDraw =
        new double[] {
          Math.abs(m_elevatorSim.getCurrentDrawAmps()), Math.abs(m_elevatorSim.getCurrentDrawAmps())
        };
    inputs.heightMeters = m_elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.velocityRadPerSec = inputs.velocityMetersPerSec / PeriscopeConstants.DRUM_RADIUS_M;
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage = MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_elevatorSim.setInputVoltage(m_voltage);
  }
}
