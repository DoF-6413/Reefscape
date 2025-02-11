package frc.robot.Subsystems.Periscoper;

import edu.wpi.first.math.util.Units;

public class PeriscoperConstants {
    
    public static final double GEAR_RATIO = 12.0/42.0;
    public static final double SPOOL_RADIUS_M = Units.inchesToMeters(0.625);
    public static final double MAX_HEIGHT_M = Units.inchesToMeters(58); //meters
    public static final double MOI = 0.0; //TODO: Find MOI
    public static final boolean ENABLE_CUR_LIM = true; 
    public static final int CUR_LIM_A = 60;
    public static final double MAX_VELOCITY_MPS = 1.4; //meters per second
    public static final double MASS_LBS = 30.0; //pounds
    public static final double MAX_ACCELERATION_MPS2 = 0.0; //meters per second squared TODO: Find max acceleration
    public static final int CANID_1 = 15; // 
    public static final int CANID_2 = 16;
    public static final int UPDATE_FREQUENCY_HZ = 50;
    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KG = 0.0;
}
