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

  // Controllers
  private final ProfiledPIDController m_profiledPIDController;
  private ElevatorFeedforward m_elevatorFeedforward;
  private double m_setpointMeters = 0.0;

  /**
   * This constructs a new {@link PeriscopeIOSim} instance.
   *
   * <p>This creates a new {@link PeriscopeIO} object that uses two simulated KrakenX60 motors to
   * drive the simulated Periscope (elevator) mechanism
   */
  public PeriscopeIOSim() {
    System.out.println("[Init] Creating PeriscopeIOSim");

    // Initailize elevator simulation
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

    // Initailize controllers with gains
    m_profiledPIDController =
        new ProfiledPIDController(
            PeriscopeConstants.KP,
            PeriscopeConstants.KI,
            PeriscopeConstants.KD,
            new TrapezoidProfile.Constraints(
                PeriscopeConstants.MAX_VELOCITY_ROT_PER_SEC,
                PeriscopeConstants.IDEAL_ACCELERATION_ROT_PER_SEC2));
    m_elevatorFeedforward =
        new ElevatorFeedforward(
            PeriscopeConstants.KS,
            PeriscopeConstants.KG,
            PeriscopeConstants.KV,
            PeriscopeConstants.KA);
  }

  @Override
  public void updateInputs(PeriscopeIOInputs inputs) {
    // Calculate next output voltage from Profiled PID and Feedforward controllers
    double voltage =
        m_profiledPIDController.calculate(inputs.heightMeters, m_setpointMeters)
            + m_elevatorFeedforward.calculate(m_profiledPIDController.getGoal().velocity);
    this.setVoltage(voltage);

    // Update elevator sim
    m_elevatorSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update inputs
    inputs.isConnected = new boolean[] {true, true};
    inputs.heightMeters = m_elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.velocityRadPerSec =
        inputs.velocityMetersPerSec / PeriscopeConstants.DRUM_RADIUS_M;
    inputs.appliedVolts = new double[] {voltage, voltage};
    inputs.currentDraw =
        new double[] {
          Math.abs(m_elevatorSim.getCurrentDrawAmps()), Math.abs(m_elevatorSim.getCurrentDrawAmps())
        };
  }

  @Override
  public void setVoltage(double volts) {
    m_elevatorSim.setInputVoltage(
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE));
  }

  @Override
  public void setPosition(double heightMeters) {
    m_setpointMeters = heightMeters;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_profiledPIDController.setPID(kP, kI, kD);
  }

  @Override
  public void setFF(double kS, double kG, double kV, double kA) {
    m_elevatorFeedforward.setKs(kS);
    m_elevatorFeedforward.setKg(kG);
    m_elevatorFeedforward.setKv(kV);
    m_elevatorFeedforward.setKa(kA);
  }
}
