package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase{

    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
  
    /** Creates a new funnel. */
    public Funnel(FunnelIO funnelIO) {
      //initialize the funnel subsystem
      System.out.println("[Init] Creating Funnel");
      this.io = funnelIO;
  
  
    }
  
    @Override
    // This method will be called once per scheduler run
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Funnel", inputs);
    }
  
    public void setVoltage(double volts){
      io.setVoltage(volts);
    }
  }