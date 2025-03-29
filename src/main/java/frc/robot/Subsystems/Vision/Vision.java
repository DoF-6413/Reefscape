// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.Drive.Drive;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final VisionConsumer m_consumer;

  // Vision pose estimation
  private final PhotonPoseEstimator[] m_photonPoseEstimators;
  private List<Pose2d> m_estimatedPoses = new LinkedList<>();
  private Matrix<N3, N1> m_stdDevs = VecBuilder.fill(0.5, 0.5, 1000000);

  /**
   * Constructs a new {@link Vision} instance.
   *
   * <p>This creates a new Vision {@link SubsystemBase} object that updates the pose of the robot
   * based on AprilTag readings from cameras.
   *
   * @param Consumer Used to pass in Vision estimated Pose into {@link Drive} subsystem's {@link
   *     SwerveDrivePoseEstimator}.
   * @param io {@link VisionIO} implementation(s) of the different cameras which determines whether
   *     the methods and inputs are initialized with the real, sim, or replay code.
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");

    // Initialize IO, and consumer
    m_io = io;
    m_consumer = consumer;

    // Initialize loggers and Vision Pose Estimators based on number of cameras
    m_inputs = new VisionIOInputsAutoLogged[m_io.length];
    m_photonPoseEstimators = new PhotonPoseEstimator[m_io.length];
    for (int i = 0; i < m_io.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
      m_photonPoseEstimators[i] =
          new PhotonPoseEstimator(
              FieldConstants.APRILTAG_FIELD_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              VisionConstants.CAMERA_ROBOT_OFFSETS[i]);
      m_photonPoseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      Logger.recordOutput(
          "Camera/" + VisionConstants.CAMERA_NAMES[i], VisionConstants.CAMERA_ROBOT_OFFSETS[i]);
    }
  }

  @Override
  public void periodic() {
    // Update logger and check for any AprilTags for each camera
    for (int i = 0; i < m_inputs.length; i++) {
      // Update and log inputs
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/" + VisionConstants.CAMERA_NAMES[i], m_inputs[i]);

      // Check results and add available and unambiguous Vision measurements to list
      var currentResult = m_inputs[i].pipelineResult;
      if (!currentResult.hasTargets())
        continue; // Move to next camera update iteration if no AprilTags seen
      var optionalEstimatedPose = m_photonPoseEstimators[i].update(currentResult);
      if (optionalEstimatedPose.isEmpty())
        continue; // Move to next camera update iteration if no position is estimated
      var estimatedPose = optionalEstimatedPose.get().estimatedPose.toPose2d();
      double ambiguity =
          (currentResult.targets.size() == 1)
              ? currentResult.getBestTarget().getPoseAmbiguity()
              : currentResult.getMultiTagResult().get().estimatedPose.ambiguity;
      if (
      // Ensure pose is trustworthy and within field bounds in order to be used
      ambiguity >= 0.0
          && ambiguity <= 0.2
          && estimatedPose.getX() >= 0.0
          && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH
          && estimatedPose.getY() >= 0.0
          && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH) {

        m_estimatedPoses.add(estimatedPose);
        // Record estimated pose
        Logger.recordOutput(
            "Odometry/Vision/EstimatedPoses/" + VisionConstants.CAMERA_NAMES[i], estimatedPose);
      }
    }

    if (m_estimatedPoses.size() == 0)
      return; // Move to next periodic iteration if no poses estimated

    /* Add Vision measurements to Swerve Pose Estimator in Drive through the VisionConsumer */
    if (m_estimatedPoses.size() > 1) {
      // Average poses is both cameras see an AprilTag and clear pose list
      var averagePose =
          averageVisionPoses(m_estimatedPoses.toArray(new Pose2d[m_estimatedPoses.size()]));
      m_consumer.accept(averagePose, m_inputs[0].timestampSec, m_stdDevs);
      m_estimatedPoses.clear();
    } else {
      // Use pose generated from the camera that saw an AprilTag and clear pose list
      m_consumer.accept(m_estimatedPoses.get(0), m_inputs[0].timestampSec, m_stdDevs);
      m_estimatedPoses.clear();
    }

    // Update pose estimator from limelight
    // if (RobotStateConstants.getMode() != RobotStateConstants.Mode.SIM) {
    //   var estimatedPose = m_inputs[0].limelightPose;
    //   if (estimatedPose == null) return;
    //   m_consumer.accept(estimatedPose, Timer.getFPGATimestamp(), m_stdDevs);
    // }
  }

  /**
   * Retrieves the latest pipeline and checks if an AprilTag is seen to determine the ID returned.
   *
   * @param index Camera index.
   * @return ID of AprilTag currently seen, -1 if none seen.
   */
  public int getTagID(int index) {
    var result = m_inputs[index].pipelineResult;
    if (!result.hasTargets()) return -1;
    return result.getBestTarget().getFiducialId();
  }

  /**
   * Calculates the average position between the Estimated Poses from the Vision.
   *
   * @param estimatedPoses Poses to average.
   * @return Pose2d with the averaged position.
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
     * Passes in inputed values to Swerve Pose Estimator in Drive.
     *
     * @param visionRobotPose 2d pose calculated from AprilTag.
     * @param timestampSec Timestamp when position was calculated in seconds.
     * @param visionStdDevs Standard deviation from the average calculation (distance & angle).
     */
    public void accept(Pose2d visionRobotPose, double timestampSec, Matrix<N3, N1> visionStdDevs);
  }
}
