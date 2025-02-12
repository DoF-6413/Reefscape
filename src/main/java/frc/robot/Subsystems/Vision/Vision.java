// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final VisionConsumer m_consumer;
  private final PhotonPoseEstimator[] m_photonPoseEstimators;
  List<Pose2d> estimatedPoses = new LinkedList<>();

  /**
   * Constructs a new Vision subsystem instance.
   *
   * <p>This constructor creates a new Vision object that updates the pose of the robot based on
   * camera readings
   *
   * @param Consumer Used to pass in Vision estimated Pose into Drive subsystem's Swerve Pose
   *     Estimator
   * @param io VisionIO implementation of the current robot mode (only real or blank)
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");
    m_io = io;
    m_inputs = new VisionIOInputsAutoLogged[m_io.length];
    m_photonPoseEstimators = new PhotonPoseEstimator[m_io.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
      m_photonPoseEstimators[i] =
          new PhotonPoseEstimator(
              VisionConstants.APRILTAG_FIELD_LAYOUT,
              PoseStrategy.LOWEST_AMBIGUITY,
              VisionConstants.CAMERA_ROBOT_OFFSETS[i]);
    }
    m_consumer = consumer;
  }

  // TOOO: Take ambiguity of last 3 frames and average it to avoid false 0.0 ambiguity reports
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < m_inputs.length; i++) {
      // Update and log inputs
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/" + VisionConstants.CAMERA_NAMES[i], m_inputs[i]);

      // Check results and add possible Vision measurements
      var result = getPipelineResult(i);
      if (!result.hasTargets()) continue;
      var target = result.getBestTarget();
      SmartDashboard.putNumber("AprilTagID", target.getFiducialId());
      if (target.getFiducialId() >= 1
          && target.getFiducialId() <= 22
          && target.getPoseAmbiguity() >= 0.0
          && target.getPoseAmbiguity() <= 0.2) {
        var estimatedPose = m_photonPoseEstimators[i].update(result);
        if (estimatedPose.isEmpty()) continue;
        estimatedPoses.add(estimatedPose.get().estimatedPose.toPose2d());
      }
    }

    if (estimatedPoses.size() == 0) {
      estimatedPoses.clear();
      return;
    }
    Logger.recordOutput(
        "Vision/EstimatedPoses", estimatedPoses.toArray(new Pose2d[estimatedPoses.size()]));
    if (estimatedPoses.size() > 1) {
      var averagePose =
          averageVisionPoses(estimatedPoses.toArray(new Pose2d[estimatedPoses.size()]));
      m_consumer.accept(averagePose, Timer.getFPGATimestamp());
      estimatedPoses.clear();
    } else {
      m_consumer.accept(estimatedPoses.get(0), Timer.getFPGATimestamp());
      estimatedPoses.clear();
    }
  }

  public PhotonPipelineResult getPipelineResult(int index) {
    return m_inputs[index].pipelineResult;
  }

  public int getTagID() {
    var result = this.getPipelineResult(0);
    if (!result.hasTargets()) return -1;
    return result.getBestTarget().getFiducialId();
  }

  public Pose2d toAprilTag() {
    var id = this.getTagID();
    if (id < 1 || id > 22) {
      return new Pose2d(-1, -1, new Rotation2d());
    }

    var tagPose2d = VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(id).get().toPose2d();
    var inFrontOfTag =
        new Pose2d(
            tagPose2d.getX()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getCos(),
            tagPose2d.getY()
                + ((DriveConstants.TRACK_WIDTH_M / 2) + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getSin(),
            tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
    return inFrontOfTag;
  }

  /**
   * Calculates the average position between the Estimated Poses from the Vision
   *
   * @param estimatedPoses Poses to average
   * @return Pose2d with the averaged position
   */
  private Pose2d averageVisionPoses(Pose2d... estimatedPoses) {
    double x = 0;
    double y = 0;
    double theta = 0;
    for (Pose2d pose : estimatedPoses) {
      x += pose.getX();
      y += pose.getY();
      theta += pose.getRotation().getRadians();
    }

    // Averages x, y and theta components and returns the values in a new Pose2d
    return new Pose2d(
        new Translation2d(x / estimatedPoses.length, y / estimatedPoses.length),
        new Rotation2d(theta / estimatedPoses.length));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds);
  }
}
