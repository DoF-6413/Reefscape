package frc.robot.Subsystems.Gyro;

public class GyroConstants {
  /**
   * An angle to offset the heading of the Gyro to be the desired Front side of the robot. The Front
   * side will be defined as the scoring side for this robot
   */
  public static final double HEADING_OFFSET_RAD = Math.PI;
  /** CAN ID for the Pigeon 2.0 IMU */
  public static final int CAN_ID = 14;
  /** Update the signals from the Pigeon IMU 100 times a second (every 0.01 seconds) */
  public static final double UPDATE_FREQUENCY_HZ = 100.0;
}
