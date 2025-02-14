// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
   * @param io VisionIO implementation(s) of different cameras for the current robot mode (real or
   *     sim)
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");

    m_consumer = consumer;
    m_io = io;
    m_inputs = new VisionIOInputsAutoLogged[m_io.length];
    m_photonPoseEstimators = new PhotonPoseEstimator[m_io.length];

    // Initilize loggers and Vision pose estimators based on number of cameras
    for (int i = 0; i < m_io.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
      m_photonPoseEstimators[i] =
          new PhotonPoseEstimator(
              VisionConstants.APRILTAG_FIELD_LAYOUT,
              PoseStrategy.LOWEST_AMBIGUITY,
              VisionConstants.CAMERA_ROBOT_OFFSETS[i]);
    }
  }

  // TODO: Take ambiguity of last 3 frames and average it to avoid false 0.0 ambiguity reports
  @Override
  public void periodic() {
    for (int i = 0; i < m_inputs.length; i++) {
      // Update and log inputs
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/" + VisionConstants.CAMERA_NAMES[i], m_inputs[i]);

      // Check results and add possible Vision measurements
      var result = getPipelineResult(i);
      if (!result.hasTargets()) continue; // Go to next iteration if no AprilTags seen
      var target = result.getBestTarget();
      if (target.getFiducialId() >= 1
          && target.getFiducialId() <= 22
          && target.getPoseAmbiguity() >= 1e-6
          && target.getPoseAmbiguity() <= 0.2) {
        var estimatedPose = m_photonPoseEstimators[i].update(result);
        if (estimatedPose.isEmpty()) continue; // Go to next iteration if no position is estimated
        estimatedPoses.add(estimatedPose.get().estimatedPose.toPose2d());
      }
    }

    /* Add Vision measurments to Swerve Pose Estimator in Drive through the VisionConsumer */
    if (estimatedPoses.size() == 0) return; // Stop here if no poses estimated

    // Log estimated poses, under "RealOutputs" tab rather than "AdvantageKit" for some reason
    Logger.recordOutput(
        "Vision/EstimatedPoses", estimatedPoses.toArray(new Pose2d[estimatedPoses.size()]));

    if (estimatedPoses.size() > 1) {
      // Average poses is both cameras see an AprilTag and clear pose list
      var averagePose =
          averageVisionPoses(estimatedPoses.toArray(new Pose2d[estimatedPoses.size()]));
      m_consumer.accept(averagePose, Timer.getFPGATimestamp());
      estimatedPoses.clear();
    } else {
      // Use pose generated from the camera that saw an AprilTag and clear pose list
      m_consumer.accept(estimatedPoses.get(0), Timer.getFPGATimestamp());
      estimatedPoses.clear();
    }
  }

  /**
   * @param index Camera index
   * @return PhotonPipelineResult containing latest data calculated by PhotonVision
   */
  public PhotonPipelineResult getPipelineResult(int index) {
    return m_inputs[index].pipelineResult;
  }

  /**
   * Retrieves the latest pipeline and checks if an AprilTag is seen to determine the ID returned
   *
   * @param index Camera index
   * @return ID of AprilTag currently seen, -1 if none seen
   */
  public int getTagID(int index) {
    var result = this.getPipelineResult(index);
    if (!result.hasTargets()) return -1;
    return result.getBestTarget().getFiducialId();
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
    /**
     * Passes in inputed values to Swerve Pose Estimator in Drive
     *
     * @param visionRobotPose 2d pose calculated from AprilTag
     * @param timestampSec Timestamp when position was calculated in seconds
     */
    public void accept(Pose2d visionRobotPose, double timestampSec);
  }
}
