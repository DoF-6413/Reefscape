// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim extends VisionIOPhotonVision {
  private final VisionSystemSim m_sim;
  private final PhotonCameraSim m_cameraSim;
  private Supplier<Pose2d> m_currentPose;

  public VisionIOSim(int index, Supplier<Pose2d> currentPose) {
    super(index);

    var camProp = new SimCameraProperties();
    camProp.setAvgLatencyMs(20);
    camProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
    camProp.setFPS(40);
    m_cameraSim = new PhotonCameraSim(m_camera, camProp);

    m_sim = new VisionSystemSim(VisionConstants.CAMERA_NAMES[index]);
    m_sim.addAprilTags(VisionConstants.APRILTAG_FIELD_LAYOUT);
    m_sim.addCamera(m_cameraSim, m_cameraOffset);

    m_currentPose = currentPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_sim.update(m_currentPose.get());
    super.updateInputs(inputs);
  }
}
