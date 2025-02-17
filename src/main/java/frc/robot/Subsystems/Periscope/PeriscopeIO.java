package frc.robot.Subsystems.Periscope;

import org.littletonrobotics.junction.AutoLog;

public interface PeriscopeIO {

  @AutoLog
  public static class PeriscopeIOInputs {
    public double[] appliedVolts = {0.0, 0.0};
    public double[] currentDraw = {0.0, 0.0};
    public double[] tempCelsius = {0.0, 0.0};
    public double heightMeters = 0.0;
    public double velocityRadPerSec = 0.0;
    public boolean[] isConnected = {false, false};
  }

  public default void updateInputs(PeriscopeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double heightMeters) {}

  public default void setPID(double kP, double kI, double kD) {}
}
