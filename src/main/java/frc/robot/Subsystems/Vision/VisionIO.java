package frc.robot.Subsystems.Vision;

import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    /** Measurments and other data from AprilTags if one is seen */
    public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    /** If the camera sees an AprilTag */
    public boolean hasTargets = false;
    /** Timestamp when Pipeline was recieved */
    public double timestampSec = 0.0;
    /** AprilTag tracked by the camera */
    public PhotonTrackedTarget target = null;
    /** ID number associated with AprilTag */
    public int fiducialID = 0;
    /**
     * Ratio of trustworthiness for calculated pose from AprilTag. Lower value means more
     * trustworthy and anything > 0.2 shouldn't be used
     */
    public double poseAmbiguity = 0.0;
  }

  /**
   * Updates the logged inputs for a camera in the Vision system. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * @return A list of all PhotonPipelineResults waiting in queue
   */
  public default List<PhotonPipelineResult> getAllPipelineResults() {
    return null;
  }
}
