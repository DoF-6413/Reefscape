package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

  // Controller
  private final PIDController m_PIDController;
  private boolean m_enablePID = false;

  /**
   * Constructs a new {@link Climber} instance.
   *
   * <p>This creates a new Climber {@link SubsystemBase} object with the given IO implementation
   * which determines whether the methods and inputs are initialized with the real, sim, or replay
   * code.
   *
   * @param io {@link ClimberIO} implementation of the current robot mode.
   */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");

    // Initialize the IO implementation
    m_io = io;

    // Initialize PID controller
    m_PIDController =
        new PIDController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);

    // Tunable PID gains
    SmartDashboard.putBoolean("PIDFF_Tuning/Climber/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF_Tuning/Climber/KP", ClimberConstants.KP);
    SmartDashboard.putNumber("PIDFF_Tuning/Climber/KI", ClimberConstants.KI);
    SmartDashboard.putNumber("PIDFF_Tuning/Climber/KD", ClimberConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Climber", m_inputs);

    if (m_enablePID) {
      // Calculate voltage from PID controller
      this.setVoltage(m_PIDController.calculate(m_inputs.positionRad));

      // Enable and update tunable PID gains through SmartDashboard
      if (SmartDashboard.getBoolean("PIDFF_Tuning/Climber/EnableTuning", false)) {
        this.updatePID();
      }
    }
  }

  /**
   * Sets the idle mode of the Climber motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the Climber motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the position of the Climber using a PID controller.
   *
   * @param positionRad Angular position of the Climber in radians.
   */
  public void setAngle(double positionRad) {
    Logger.recordOutput("Superstructure/Setpoints/ClimberAngle", positionRad);
    m_PIDController.setSetpoint(positionRad);
  }

  /**
   * Sets the PID gains of the Climber motor's PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public void setPID(double kP, double kI, double kD) {
    m_PIDController.setPID(kP, kI, kD);
  }

  /** Update PID gains for the Climber motor from SmartDashboard inputs. */
  private void updatePID() {
    // If any value on SmartDashboard changes, update the gains
    if (ClimberConstants.KP
            != SmartDashboard.getNumber("PIDFF_Tuning/Climber/KP", ClimberConstants.KP)
        || ClimberConstants.KI
            != SmartDashboard.getNumber("PIDFF_Tuning/Climber/KI", ClimberConstants.KI)
        || ClimberConstants.KD
            != SmartDashboard.getNumber("PIDFF_Tuning/Climber/KD", ClimberConstants.KD)) {
      ClimberConstants.KP =
          SmartDashboard.getNumber("PIDFF_Tuning/Climber/KP", ClimberConstants.KP);
      ClimberConstants.KI =
          SmartDashboard.getNumber("PIDFF_Tuning/Climber/KI", ClimberConstants.KI);
      ClimberConstants.KD =
          SmartDashboard.getNumber("PIDFF_Tuning/Climber/KD", ClimberConstants.KD);
      // Sets the new gains
      this.setPID(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);
    }
  }
}
