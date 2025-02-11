package frc.robot.Subsystems.Periscoper;

import org.littletonrobotics.junction.AutoLog;

public interface PeriscoperIO {

    @AutoLog
    public static class PeriscoperIOInputs {
        public double[] appliedVolts = {0.0, 0.0};
        public double[] currentDraw = {0.0, 0.0};
        public double[] tempCelsius = {0.0, 0.0};
        public double heightMeters = 0.0;
        public double velocityRadPerSec = 0.0;
        public boolean[] isConnected = {false, false};

    }

    public default void updateInputs(PeriscoperIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocity) {} 
}
