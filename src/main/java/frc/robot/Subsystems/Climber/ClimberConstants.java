package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
    public static final int CAN_ID = 17; //CAN ID for climb motor
    //fake constants TODO: find real values and add more
    public static final double GEAR_RATIO = 9/1;
    public static  double KP = 0;
    public static  double KI = 0;
    public static  double KD = 0;
    public static final double UPDATE_FREQUENCY_HZ = 100;
    public static final int CUR_LIM_A = 50;
    public static final boolean ENABLE_CUR_LIM = true;
    public static final double CLIMBER_MIN_ANGLE_RAD = 0;
    public static final double CLIMBER_MAX_ANGLE_RAD = Units.degreesToRadians(80);
    public static final double MOI_KG_M2 = 0;
    public static final double LENGHT_M = Units.inchesToMeters(11.919);
    
}
