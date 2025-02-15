package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constant values for the Vision subsystem. Index 0 refers to the Front camera (scoring side) and
 * index 1 refers to the Back camera (intake side)
 */
public class VisionConstants {
  public enum CAMERA {
    FRONT(0),
    BACK(1);

    public final int CAMERA_INDEX;

    CAMERA(int value) {
      CAMERA_INDEX = value;
    }
  }

  /** Names of cameras on PhotonVision and NetworkTables */
  public static final String[] CAMERA_NAMES = {"Front", "Back"};

  /** 3d offset of the center of the robot to the Front camera */
  private static final Transform3d FRONT_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, 0, 0));
  /** 3d offset of the center of the robot to the Back camera */
  private static final Transform3d BACK_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, Units.degreesToRadians(35), Math.PI));
  /** Array of 3d transformations from the center of the robot to each camera location */
  public static final Transform3d[] CAMERA_ROBOT_OFFSETS = {
    FRONT_CAMERA_ROBOT_OFFSET, BACK_CAMERA_ROBOT_OFFSET
  };

  /** Baseline standard deviation for proccessed AprilTag translation in meters */
  public static final double LINEAR_STD_DEV_M = 0.1; // TODO: Test and update
  /** Baseline standard deviation for proccessed AprilTag rotation in radians */
  public static final double ANGULAR_STD_DEV_RAD = 0.1; // TODO: Test and update

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
  /** Average pixel error in calibarion */
  public static final double AVERAGE_ERROR_PX =
      0.0; // TODO: Update with real value from camera calibration
  /** Standard deviations of pixel error */
  public static final double ERROR_STDDEV_PX =
      0.0; // TODO: Update with real value from camera calibration
  /**
   * Display simulated camera feed. Tanks performance a fair bit on STEM center laptops (TODO:
   * verify and set to false if needed)
   */
  public static final boolean ENABLE_SIM_CAMERA_STREAM = true;
}
