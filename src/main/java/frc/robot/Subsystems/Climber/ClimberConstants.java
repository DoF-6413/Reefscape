package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  //CAN ID for climber motor
  public static final int CAN_ID = 17; 
  // gear ratio of 9:1 for the climber motor
  public static final double GEAR_RATIO = 9.0 / 1.0;
  public static final double UPDATE_FREQUENCY_HZ = 100;
  //current limit, in amps, for the climber motor
  public static final int CUR_LIM_A = 50;
  public static final boolean ENABLE_CUR_LIM = true;
  public static final double CLIMBER_MIN_ANGLE_RAD = Units.degreesToRadians(10);
  public static final double CLIMBER_MAX_ANGLE_RAD = Units.degreesToRadians(90);
  public static final double LENGTH_M = Units.inchesToMeters(11.919);
  
  //PID Constants
  /**
   * KP represents the constant multiplied by the current error from setpoint (Proportional Error)
   */
  public static double KP = 1.0;
  /**
   * KI represents the constant multiplied by the integral of the error from setpoint (Integral
   * Error)
   */
  public static double KI = 0.0;
  /**
   * KD represents the constant multiplied by the change in error over time (Derivative Error) 
   */
  public static double KD = 0.0;

  //SIM Constants
  /**Moment of inertia of the arm in kilograms * meters squared*/
  public static final double MOI_KG_M2 = 0.0000001; //TODO: get the MOI 
}
