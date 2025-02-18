package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOSim implements ClimberIO {
  // Arm system simulation
  private final SingleJointedArmSim m_climberSim;
  private double m_voltage = 0.0;

  // PID controller
  private final PIDController m_PIDController;
  private double m_setpointRad = 0.0;

  /**
   * Constructs a new {@link ClimberIOSim} instance.
   *
   * <p>This creates a new {@link ClimberIO} object that uses a simulated KrakenX60 motor to drive
   * the simulated Climber mechanism.
   */
  public ClimberIOSim() {
    System.out.println("[Init] Creating ClimberIOSim");

    // Initialize the simulated Climber arm with a KrakenX60 motor
    m_climberSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(1), ClimberConstants.MOI_KG_M2, ClimberConstants.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.LENGTH_M,
            ClimberConstants.CLIMBER_MIN_ANGLE_RAD,
            ClimberConstants.CLIMBER_MAX_ANGLE_RAD,
            ClimberConstants.SIMULATE_GRAVITY,
            ClimberConstants.CLIMBER_MAX_ANGLE_RAD); // Starting height

    // Initialize PID controller
    m_PIDController =
        new PIDController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Calculate and apply next output voltage from the PID controller
    m_voltage = m_PIDController.calculate(inputs.positionRad, m_setpointRad);
    this.setVoltage(m_voltage);

    // Update arm sim
    m_climberSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Update logged inputs from simulated arm system
    inputs.isConnected = true;
    inputs.appliedVoltage = m_voltage;
    inputs.currentAmps = Math.abs(m_climberSim.getCurrentDrawAmps());
    inputs.positionRad = m_climberSim.getAngleRads();
    inputs.velocityRadPerSec = m_climberSim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
    m_climberSim.setInputVoltage(m_voltage);
  }

  @Override
  public void setPosition(double positionRad) {
    m_setpointRad = positionRad;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }
}
