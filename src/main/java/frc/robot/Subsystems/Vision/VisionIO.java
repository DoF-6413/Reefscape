// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    // PhotonVision result objects
    public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public PhotonTrackedTarget target = null;
    public boolean hasTargets = false;
    public int fiducialID = 0;
    public double poseAmbiguity = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
