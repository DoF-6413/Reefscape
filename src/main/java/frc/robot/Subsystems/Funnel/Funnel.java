package frc.robot.Subsystems.Funnel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO m_io;
  private final FunnelIOInputsAutoLogged m_inputs = new FunnelIOInputsAutoLogged();

  // PID Controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Creates a new {@link Funnel} instance.
   *
   * <p>This creates a new Funnel {@link SubsystemBase} object with given IO implementation which
   * determines whether the methods and inputs are initailized with the real, sim, or replay code
   *
   * @param io {@link FunnelIO} implementation of the current mode of the robot
   */
  public Funnel(FunnelIO io) {
    // Initialize Funnel subsystem
    System.out.println("[Init] Creating Funnel");

    // Initialize the IO implementation
    m_io = io;

    /// Initialize the PID controller
    m_PIDController = new PIDController(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);

    // Tunable PID values
    SmartDashboard.putBoolean("PIDFF/Funnel/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/Funnel/KP", FunnelConstants.KP);
    SmartDashboard.putNumber("PIDFF/Funnel/KI", FunnelConstants.KI);
    SmartDashboard.putNumber("PIDFF/Funnel/KD", FunnelConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update inputs and logger
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Funnel", m_inputs);

    // Control the Funnel through the PID controller if enabled, open loop voltage control if
    // disabled
    if (m_enablePID) {
      m_PIDController.calculate(m_inputs.velocityRadPerSec);
    }

    // Enable and update tunable PID values through SmartDashboard
    if (SmartDashboard.getBoolean("PIDFF_Tuning/Funnel/EnableTuning", false)) {
      this.updatePID();
    }
  }

  /**
   * Sets voltage of the Funnel motor. The value inputed is clamped between values of -12 to 12
   *
   * @param volts A value between -12 (full reverse speed) tp 12 (full forward speed)
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the Funnel PID controller
   *
   * @param setpoint Velocity in radians per second
   */
  public void setSetpoint(double setpoint) {
    m_PIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the PID gains for PID controller
   *
   * @param kP Proportional gain value
   * @param kI Integral gain value
   * @param kD Derivative gain value
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /**
   * Enable closed loop PID control for the Funnel
   *
   * @param enable True to enable PID, false to disable
   */
  public void enablePID(boolean enable) {
    m_enablePID = enable;
  }

  /** Update PID gains for the Funnel motors from SmartDashboard inputs */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (FunnelConstants.KP != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KP", FunnelConstants.KP)
        || FunnelConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KI", FunnelConstants.KI)
        || FunnelConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KD", FunnelConstants.KD)) {
      FunnelConstants.KP = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KP", FunnelConstants.KP);
      FunnelConstants.KI = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KI", FunnelConstants.KI);
      FunnelConstants.KD = SmartDashboard.getNumber("PIDFF_Tuning/Funnel/KD", FunnelConstants.KD);
      // Sets the new gains
      this.setPID(FunnelConstants.KP, FunnelConstants.KI, FunnelConstants.KD);
    }
  }
}
