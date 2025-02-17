package frc.robot.Subsystems.Gyro;

public class GyroConstants {
  /**
   * An angle to offset the heading of the Gyro to be the desired Front side of the robot. The Front
   * side will be defined as the scoring side for this robot
   */
  public static final double HEADING_OFFSET_RAD = 0;
  /** CAN ID for the Pigeon2 Gyro */
  public static final int CAN_ID = 14;
  /** Number of times a second a signal is sent, resulting in one every 0.01 seconds */
  public static final double UPDATE_FREQUENCY_HZ = 100.0;
}
