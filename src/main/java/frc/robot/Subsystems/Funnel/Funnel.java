package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO m_io;
  private final FunnelIOInputsAutoLogged m_inputs = new FunnelIOInputsAutoLogged();

  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Creates a new Funnel Subsystem instances.
   *
   * <p>This constructor creates a new Funnel subsystem object with given IO implementation._sim
   *
   * @param io FunnelIO implementation of the current mode of the robot
   */
  public Funnel(FunnelIO io) {
    // Initialize the Funnel subsystem
    System.out.println("[Init] Creating Funnel");
    this.m_io = io;

    m_PIDController = new PIDController(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);

    // Tunable PID values
    SmartDashboard.putBoolean("PIDFF/Funnel/KP", false);
    SmartDashboard.putNumber("PIDFF/Funnel/KP", FunnelConstants.KP);
    SmartDashboard.putNumber("PIDFF/Funnel/KI", FunnelConstants.KI);
    SmartDashboard.putNumber("PIDFF/Funnel/KD", FunnelConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Funnel", m_inputs);

    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }
  }

  /**
   * Sets voltage of the Funnel motor
   *
   * @param volts A value between -12 (full reverse speed) tp 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the velocity, in radians per second, of the Funnel PID controller
   *
   * @param setPoint Velocity in radians per second
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID values for the Funnel motor's PID controller in code
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable PID for the Funnel
   *
   * @param enable True to enable PID, false to disable
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }
}
