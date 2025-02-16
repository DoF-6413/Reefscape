package frc.robot.Subsystems.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AEE extends SubsystemBase {

  private final AEEIO m_io;
  private final AEEIOInputsAutoLogged m_inputs = new AEEIOInputsAutoLogged();

  /** Creates a new CEE. */
  public AEE(AEEIO AEEIO) {
    // initialize the AEE subsystem
    System.out.println("[Init] Creating AEE");
    this.m_io = AEEIO;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("AEE", m_inputs);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }
}
