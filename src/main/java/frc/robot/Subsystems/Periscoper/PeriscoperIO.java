package frc.robot.Subsystems.Periscoper;

public interface PeriscoperIO {

    public static class PeriscoperInputs {
        public double[] periscoperAppliedVolts = {0.0, 0.0};
        public double[] periscoperCurrentDraw = {0.0, 0.0};
        public double[] periscoperTempCelcius = {0.0, 0.0};
        public double periscoperHeightMeters = 0.0;
        public double periscoperVelocityRadPerSec = 0.0;
        public boolean[] periscoperIsConnected = 

    }

    public default void updateInputs(PeriscoperInputs inputs) {}
} 
