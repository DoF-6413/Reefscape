package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO ClimberIO) {
    // initialize the Climber subsystem
    System.out.println("[Init] Creating Climber");
    this.io = ClimberIO;

    SmartDashboard.putBoolean("PIDFF/climber/EnableTuning", false);
    SmartDashboard.putNumber("PIDFF/climber/KP", ClimberConstants.KP);
    SmartDashboard.putNumber("PIDFF/climber/KI", ClimberConstants.KI);
    SmartDashboard.putNumber("PIDFF/climber/KD", ClimberConstants.KD);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    Logger.processInputs("Climber", inputs);
    io.updateInputs(inputs);
    if (SmartDashboard.getBoolean("PIDFF/Periscoper/EnableTuning", false)) {
      this.updateClimberPID();
    }
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setPID(double kP, double kI, double kD) {

    io.setPID(kP, kI, kD);
  }

  private void updateClimberPID() {
    if (ClimberConstants.KP != SmartDashboard.getNumber("PIDFF/Climber/KP", ClimberConstants.KP)
        || ClimberConstants.KI != SmartDashboard.getNumber("PIDFF/Climber/KI", ClimberConstants.KI)
        || ClimberConstants.KD
            != SmartDashboard.getNumber("PIDFF/Climber/KD", ClimberConstants.KD)) {
      ClimberConstants.KP = SmartDashboard.getNumber("PIDFF/Climber/KP", ClimberConstants.KP);
      ClimberConstants.KI = SmartDashboard.getNumber("PIDFF/Climber/KI", ClimberConstants.KI);
      ClimberConstants.KD = SmartDashboard.getNumber("PIDFF/Climber/KD", ClimberConstants.KD);
      this.setPID(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);
    }
  }
}
