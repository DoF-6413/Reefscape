package frc.robot.Subsystems.Vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera m_camera;

  /**
   * Constructs a new {@link VisionIOPhotonVision} instance.
   *
   * <p>This creates a new {@link VisionIO} object that uses a real Spinel OV9281 camera connected
   * to a Raspberry Pi 5 running PhotonVision (2025.1.1). Index 0 corresponds to the Front camera
   * and index 1 corresponds to the Back camera.
   *
   * @param index Number corresponding to camera that is to be initilized (0 - Front, 1 - Back)
   */
  public VisionIOPhotonVision(int index) {
    System.out.println(
        "[Init] Creating VisionIOPhotonVision " + VisionConstants.CAMERA_NAMES[index]);

    // Initialize camera
    m_camera = new PhotonCamera(VisionConstants.CAMERA_NAMES[index]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update inputs with every results in queue
    for (var result : m_camera.getAllUnreadResults()) {
      inputs.pipelineResult = result;
      inputs.hasTargets = result.hasTargets();
      inputs.timestampSec = result.getTimestampSeconds();
      if (inputs.hasTargets) {
        // Update values with best target seen
        inputs.target = result.getBestTarget();
        inputs.fiducialID = result.getBestTarget().getFiducialId();
        inputs.poseAmbiguity = result.getBestTarget().getPoseAmbiguity();
      } else {
        // Update values to default if no AprilTag is seen
        inputs.target = null;
        inputs.fiducialID = 0;
        inputs.poseAmbiguity = 0.0;
      }
    }
  }

  @Override
  public List<PhotonPipelineResult> getAllPipelineResults() {
    return m_camera.getAllUnreadResults();
  }
}
