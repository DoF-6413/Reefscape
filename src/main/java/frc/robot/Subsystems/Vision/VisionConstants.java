package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.Map;

/**
 * Constant values for the Vision subsystem. Index 0 refers to the Front Left camera (on Module 0),
 * index 1 refers to the Front Right camera (on Module 1), and index 2 refers to the Limelight
 */
public class VisionConstants {
  public enum CAMERA {
    LEFT(0),
    RIGHT(1),
    LIMELIGHT(2);

    public final int CAMERA_INDEX;

    CAMERA(int value) {
      CAMERA_INDEX = value;
    }
  }

  /** Names of cameras on PhotonVision and NetworkTables */
  public static final String[] CAMERA_NAMES = {"Front_Left", "Front_Right", "limelight"};

  /**
   * 3d offset of the center of the robot to the Front Left camera.
   *
   * <p>WPI coordinate system z is camera y offset and vice versa. Same for pitch and yaw
   */
  private static final Transform3d LEFT_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.447),
              Units.inchesToMeters(11.297),
              Units.inchesToMeters(6.234)),
          new Rotation3d(0, 0, Units.degreesToRadians(-15)));
  /**
   * 3d offset of the center of the robot to the Front Right camera.
   *
   * <p>WPI coordinate system z is camera offset
   */
  private static final Transform3d RIGHT_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12.447),
              Units.inchesToMeters(-10.297),
              Units.inchesToMeters(6.234)),
          new Rotation3d(0, 0, Units.degreesToRadians(15)));

  /**
   * 3d offset from the center of the robot to the limelight.
   *
   * <p>Positive x is front, positive y is right, positive z is up
   */
  private static final Transform3d LIMELIGHT_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13), Units.inchesToMeters(-4.5), Units.inchesToMeters(5.5)),
          new Rotation3d(Math.PI, Units.degreesToRadians(20.42), 0));
  /** A hashmap of 3d transformations from the center of the robot to each camera location */
  public static final Map<String, Transform3d> CAMERA_OFFSETS = new HashMap<>();

  static {
    CAMERA_OFFSETS.put(CAMERA_NAMES[0], LEFT_CAMERA_ROBOT_OFFSET);
    CAMERA_OFFSETS.put(CAMERA_NAMES[1], RIGHT_CAMERA_ROBOT_OFFSET);
    CAMERA_OFFSETS.put(CAMERA_NAMES[2], LIMELIGHT_ROBOT_OFFSET);
  }

  // SIM CONSTANTS
  /** Pixel width of resolution real cameras are set to */
  public static final int CAMERA_RESOLUTION_WIDTH_PX = 1280;
  /** Pixel height of resolution real cameras are set to */
  public static final int CAMERA_RESOLUTION_HEIGHT_PX = 720;
  /** Field of View angle of the cameras as a Rotation2d */
  public static final Rotation2d CAMERA_FOV = Rotation2d.fromDegrees(90);
  /** Average frames per second processed by real Raspberry Pis */
  public static final int AVERAGE_FPS = 40;
  /** Average processing latency from real Raspberry Pis */
  public static final int AVERAGE_LATENCY_MS = 20;
  /** Display simulated camera feed. */
  public static final boolean ENABLE_SIM_CAMERA_STREAM = true;
}
