// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {

  protected final PhotonCamera m_camera;
  private PhotonPipelineResult m_prevResult = new PhotonPipelineResult();

  public VisionIOPhotonVision(int index) {
    System.out.println(
        "[Init] Creating VisionIOPhotonVision " + VisionConstants.CAMERA_NAMES[index]);

    m_camera = new PhotonCamera(VisionConstants.CAMERA_NAMES[index]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // var allResults = m_camera.getAllUnreadResults();
    // inputs.pipelineResult =
    //     allResults.isEmpty() ? m_prevResult : allResults.get(allResults.size() - 1);
    // m_prevResult = inputs.pipelineResult;
    // inputs.hasTargets = inputs.pipelineResult.hasTargets();
    // if (inputs.pipelineResult.hasTargets()) {
    //   inputs.target = inputs.pipelineResult.getBestTarget();
    //   inputs.fiducialID = inputs.target.getFiducialId();
    //   inputs.poseAmbiguity = inputs.target.getPoseAmbiguity();
    // }

    // Update logger with ALL unread results rather than just the latest (one as seen above) // TODO: Test (can be done in sim :D)
    for (var result : m_camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue; // Go to next iteration if no AprilTag seen
      inputs.pipelineResult = result;
      inputs.hasTargets = inputs.pipelineResult.hasTargets();
      if (inputs.hasTargets) {
        // Update values with best target seen
        inputs.target = inputs.pipelineResult.getBestTarget();
        inputs.fiducialID = inputs.target.getFiducialId();
        inputs.poseAmbiguity = inputs.target.getPoseAmbiguity();
      } else {
        // Update values to default if no AprilTag is seen
        inputs.target = null;
        inputs.fiducialID = 0;
        inputs.poseAmbiguity = 0.0;
      }
    }
  }
}
