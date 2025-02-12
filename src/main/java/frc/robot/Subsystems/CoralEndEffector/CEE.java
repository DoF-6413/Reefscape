package frc.robot.Subsystems.CoralEndEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CEE extends SubsystemBase{

    private final CEEIO io;
    private final CEEIOInputsAutoLogged inputs = new CEEIOInputsAutoLogged();
  
    /** Creates a new CEE. */
    public CEE(CEEIO CEEIO) {
      //initialize the CEE subsystem
      System.out.println("[Init] Creating CEE");
      this.io = CEEIO;
  
  
    }
  
    @Override
    // This method will be called once per scheduler run
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CEE", inputs);
    }
  
    public void setVoltage(double volts){
      io.setVoltage(volts);
    }
  }