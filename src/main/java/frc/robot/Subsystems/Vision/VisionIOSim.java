// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    System.out.println("[Init] Creating VisionIOSim " + VisionConstants.CAMERA_NAMES[index]);

    var camProp = new SimCameraProperties();
    camProp.setAvgLatencyMs(20);
    camProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
    camProp.setFPS(40);
    camProp.setCalibError(0, 0); // TODO: update w/ real values from real robot
    m_cameraSim = new PhotonCameraSim(super.m_camera, camProp);

    m_sim = new VisionSystemSim(VisionConstants.CAMERA_NAMES[index]);
    m_sim.addAprilTags(VisionConstants.APRILTAG_FIELD_LAYOUT);
    m_sim.addCamera(m_cameraSim, super.m_cameraOffset);
    SmartDashboard.putData("Field/Sim/" + VisionConstants.CAMERA_NAMES[index], m_sim.getDebugField());

    m_currentPose = currentPose;

    m_cameraSim.enableProcessedStream(true);
    m_cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_sim.update(m_currentPose.get());
    super.updateInputs(inputs);
  }
}
