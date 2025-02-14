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
    /** Data package from PhotonVision containing measurments and other data from AprilTags, if seen */
    public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    /** AprilTag tracked by the camera */
    public PhotonTrackedTarget target = null;
    /** If the camera sees an AprilTag */
    public boolean hasTargets = false;
    /** ID number associated with AprilTag */
    public int fiducialID = 0;
    /** Ratio of trustworthyness for calculated pose from AprilTag */
    public double poseAmbiguity = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
