// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhotonVision implements VisionIO {

  protected final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_photonPoseEstimator;
  protected final Transform3d m_cameraOffset;

  public VisionIOPhotonVision(int index) {
    switch (index) {
      case 0:
        m_cameraOffset = VisionConstants.FRONT_CAMERA_ROBOT_OFFSET;
        break;

      case 1:
        m_cameraOffset = VisionConstants.BACK_CAMERA_ROBOT_OFFSET;
        break;

      default:
        throw new RuntimeException("Invalid Camera for VisionIOPhotonVision");
    }
    m_camera = new PhotonCamera(VisionConstants.CAMERA_NAMES[index]);
    m_photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.APRILTAG_FIELD_LAYOUT, PoseStrategy.LOWEST_AMBIGUITY, m_cameraOffset);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var allResults = m_camera.getAllUnreadResults();
    inputs.pipelineResult =
        allResults.isEmpty()
            ? null
            : allResults.get(
                allResults.size()
                    - 1); // Doesnt make sense actually bc if size == 0, it'll crash either way
    // bc index == size?
    inputs.hasTargets = inputs.pipelineResult.hasTargets();
    if (inputs.hasTargets) {
      inputs.target = inputs.pipelineResult.getBestTarget();
      inputs.fiducialID = inputs.target.getFiducialId();
      inputs.poseAmbiguity = inputs.target.getPoseAmbiguity();
      inputs.estimatedPose =
          m_photonPoseEstimator.update(inputs.pipelineResult).get().estimatedPose.toPose2d();
    }
  }
}
