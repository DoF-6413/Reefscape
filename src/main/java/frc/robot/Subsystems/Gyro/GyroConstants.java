package frc.robot.Subsystems.Gyro;

public class GyroConstants {
  /**
   * Sets the offset of the heading so that the side opposite of the battery is the front of the
   * robot.
   *
   * <p>Offset = difference between gyro & rio orientation rather than gyro orientation w robot
   */
  public static final double HEADING_OFFSET_RAD = 00;
  /** Sets the ID for the Pigeon2 gyro */
  public static final int CAN_ID = 14;
  /** How many times a second a signal is sent */
  public static final double UPDATE_FREQUENCY_HZ = 100.0;
}
