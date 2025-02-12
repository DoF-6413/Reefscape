package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO ClimberIO) {
    //initialize the Climber subsystem
    System.out.println("[Init] Creating Climber");
    this.io = ClimberIO;

  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    Logger.processInputs("Climber", inputs);
    io.updateInputs(inputs);
  }

  public void setClimberVoltage(double volts){
    io.setClimberVoltage(volts);
  }
  public void setClimberVelocity(double velocity){
    io.setClimberVelocity(velocity);
}
}