package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.util.Units;

public class GyroConstants {
  // TODO: Change these values to match the robot
  /**
   * Sets the offset of the heading so that the side opposite of the battery is the front of the
   * robot
   */
  // Offset = difference between gyro & rio orientation rather than gyro orientation w robot
  public static final double HEADING_OFFSET_RAD = Units.degreesToRadians(90);
  /** Sets the ID for the Pigeon2 gyro */
  public static final int CAN_ID = 14;
  /** How many times a second a signal is sent */
  public static final double UPDATE_FREQUENCY_HZ = 100.0;
}
