// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;

  /**
   * Constructs a new Vision subsystem instance.
   *
   * <p>This constructor creates a new Vision object that updates the pose of the robot based on
   * camera readings
   *
   * @param io VisioNIO implementation of the current robot mode (only real or blank)
   */
  public Vision(VisionIO... io) {
    System.out.println("[Init] Creating Vision");
    m_io = io;
    m_inputs = new VisionIOInputsAutoLogged[m_io.length];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < m_io.length; i++) {
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision" + VisionConstants.CAMERA.values()[i], m_inputs[i]);
    }
  }

  public Pose2d getEstimatedPose(int index) {
    return m_inputs[index].estimatedPose;
  }

  public PhotonPipelineResult getPipelineResult(int index) {
    return m_inputs[index].pipelineResult;
  }
}
