package frc.robot.Subsystems.Drive;

public class GyroConstants {
  /**
   * An angle to offset the heading of the Gyro to be the desired Front side of the robot. The Front
   * side will be defined as the scoring side for this robot
   */
  public static final double HEADING_OFFSET_RAD = 0;
  /** CAN ID for the Pigeon 2.0 IMU */
  public static final int CAN_ID = 14;
  /** Refresh the signals from the Pigeon IMU 50 times a second (every 0.02 seconds) */
  public static final double UPDATE_FREQUENCY_HZ = 50.0;
  /** Refresh the position signals from the Pigeon IMU 250 times a second (every 0.004 seconds) */
  public static final double ODOMETRY_UPDATE_FREQUENCY_HZ = 250.0;
}
