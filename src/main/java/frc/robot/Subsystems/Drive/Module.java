package frc.robot.Subsystems.Drive;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io) {
    System.out.println("[Init] Creating Module");
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }
  /** Stops the Robot */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }

  public void periodic() {}
}
