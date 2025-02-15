package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim extends VisionIOPhotonVision {
  private final VisionSystemSim m_sim;
  private final PhotonCameraSim m_cameraSim;
  private Supplier<Pose2d> m_currentPose;

  /**
   * Constructs a new VisionIOSim instance
   *
   * <p>This creates a new VisionIO object that extends VisionIOPhotonVision. This is done
   * because the simulated cameras also use the PhotonCamera objects, so by making this a child
   * class the code for the PhotonCameras can be reused without needing to be redefined
   *
   * @param index Camera index
   * @param currentPose Pose2d supplier of the robot's current position from Pose Estimator
   */
  public VisionIOSim(int index, Supplier<Pose2d> currentPose) {
    super(index);
    System.out.println("[Init] Creating VisionIOSim " + VisionConstants.CAMERA_NAMES[index]);

    // Initilize simulated camera
    var camProp = new SimCameraProperties();
    camProp.setAvgLatencyMs(VisionConstants.AVERAGE_LATENCY_MS);
    camProp.setCalibration(
        VisionConstants.CAMERA_RESOLUTION_WIDTH_PX,
        VisionConstants.CAMERA_RESOLUTION_HEIGHT_PX,
        VisionConstants.CAMERA_FOV);
    camProp.setFPS(VisionConstants.AVERAGE_FPS);
    camProp.setCalibError(VisionConstants.AVERAGE_ERROR_PX, VisionConstants.ERROR_STDDEV_PX);
    m_cameraSim = new PhotonCameraSim(super.m_camera, camProp);

    // Initilze Vision system simulation
    m_sim = new VisionSystemSim(VisionConstants.CAMERA_NAMES[index]);
    m_sim.addAprilTags(FieldConstants.APRILTAG_FIELD_LAYOUT);
    m_sim.addCamera(m_cameraSim, VisionConstants.CAMERA_ROBOT_OFFSETS[index]);

    // Get current position from Pose Estimator
    m_currentPose = currentPose;

    // Enable simulated camera video streams on NetworkTables. Can be accessed through
    // SmartDashboard
    m_cameraSim.enableProcessedStream(VisionConstants.ENABLE_SIM_CAMERA_STREAM);
    m_cameraSim.enableDrawWireframe(VisionConstants.ENABLE_SIM_CAMERA_STREAM);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulated Vision system with current robot pose and update logged inputs
    m_sim.update(m_currentPose.get());
    super.updateInputs(inputs);
  }
}
