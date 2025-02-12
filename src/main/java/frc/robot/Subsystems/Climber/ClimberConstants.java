package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
    public static final int CAN_ID = 17; //CAN ID for climb motor
    //fake constants TODO: find real values and add more
    public static final double GEAR_RATIO = 9/1;
    public static final double CLIMB_KP = 0;
    public static final double CLIMB_KI = 0;
    public static final double CLIMB_KD = 0;
    public static final double CLIMB_KS = 0;
    public static final double CLIMB_KV = 0;
    public static final double CLIMB_KG = 0;
    public static final double UPDATE_FREQUENCY_HZ = 100;
    public static final int CUR_LIM_A = 50;
    public static final boolean ENABLE_CUR_LIM = true;
    public static final double CLIMBER_MIN_ANGLE_RAD = 0;
    public static final double CLIMBER_MAX_ANGLE_RAD = Units.degreesToRadians(80);
    
}
