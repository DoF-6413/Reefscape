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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final VisionConsumer m_consumer;

  // Vision pose estimation
  // private final PhotonPoseEstimator[] m_photonPoseEstimators;
  private List<Pose2d> m_estimatedPoses = new LinkedList<>();
  private Matrix<N3, N1> m_stdDevs = VecBuilder.fill(0.7, 0.7, 1000000);
  private double m_stdDevCoeff = 0.0;

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
    m_inputs[0] = new VisionIOInputsAutoLogged();
    // m_photonPoseEstimators = new PhotonPoseEstimator[m_io.length];
    // for (int i = 0; i < m_io.length; i++) {
    //   m_inputs[i] = new VisionIOInputsAutoLogged();
    //   m_photonPoseEstimators[i] =
    //       new PhotonPoseEstimator(
    //           FieldConstants.APRILTAG_FIELD_LAYOUT,
    //           PoseStrategy.LOWEST_AMBIGUITY,
    //           VisionConstants.CAMERA_ROBOT_OFFSETS[i]);
    Logger.recordOutput(
        "Camera/" + VisionConstants.CAMERA_NAMES[2], VisionConstants.CAMERA_ROBOT_OFFSETS[2]);
    // }
  }

  @Override
  public void periodic() {
    // Update logger and check for any AprilTags for each camera
    for (int i = 0; i < m_inputs.length; i++) {
      // Update and log inputs
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/" + VisionConstants.CAMERA_NAMES[2], m_inputs[i]);

      //   // Check results and add available and unambiguous Vision measurements to list
      //   var currentResult = getPipelineResult(i);
      //   if (!currentResult.hasTargets())
      //     continue; // Move to next camera update iteration if no AprilTags seen
      //   var target = currentResult.getBestTarget();
      //   if (target.getFiducialId() >= 1
      //       && target.getFiducialId() <= 22
      //       && target.getPoseAmbiguity() > 0.0
      //       && target.getPoseAmbiguity() <= 0.2) {
      //     var estimatedPose = m_photonPoseEstimators[i].update(currentResult);
      //     if (estimatedPose.isEmpty())
      //       continue; // Move to next camera update iteration if no position is estimated
      //     m_estimatedPoses.add(estimatedPose.get().estimatedPose.toPose2d());
      //     // Record estimated pose
      //     Logger.recordOutput(
      //         "Odometry/Vision/EstimatedPoses/" + VisionConstants.CAMERA_NAMES[i],
      //         estimatedPose.get().estimatedPose.toPose2d());

      //     // Calculate standard deviations for current pipeline results
      //     double averageTagDistance = 0.0;
      //     var allResults = m_io[i].getAllPipelineResults();
      //     int tagCount = allResults.size();
      //     if (allResults.size() == 0)
      //       continue; // Move to next camera update iteration if no results present
      //     for (PhotonPipelineResult result : allResults) {
      //       if (!result.hasTargets()) continue; // Move to next result iteration if no AprilTags
      // seen
      //       averageTagDistance +=
      //           Math.hypot(
      //               result.getBestTarget().getBestCameraToTarget().getX(),
      //               result.getBestTarget().getBestCameraToTarget().getY());
      //     }
      //     m_stdDevCoeff += (Math.pow(averageTagDistance, 2) / tagCount);
      // }
      // }

      // if (m_estimatedPoses.size() == 0)
      //   return; // Move to next periodic iteration if no poses estimated

      // /* Add Vision measurements to Swerve Pose Estimator in Drive through the VisionConsumer */
      // if (m_estimatedPoses.size() > 1) {
      //   // Create standard deviation matrix with averaged coefficient and reset cooefficient for
      // next
      //   // periodic iteration
      //   m_stdDevs =
      //       VecBuilder.fill(
      //           VisionConstants.LINEAR_STD_DEV_M * m_stdDevCoeff / m_inputs.length,
      //           VisionConstants.LINEAR_STD_DEV_M * m_stdDevCoeff / m_inputs.length,
      //           VisionConstants.ANGULAR_STD_DEV_RAD * m_stdDevCoeff / m_inputs.length);
      //   m_stdDevCoeff = 0.0;
      //   // Average poses is both cameras see an AprilTag and clear pose list
      //   var averagePose =
      //       averageVisionPoses(m_estimatedPoses.toArray(new Pose2d[m_estimatedPoses.size()]));
      //   m_consumer.accept(averagePose, m_inputs[0].timestampSec, m_stdDevs);
      //   m_estimatedPoses.clear();
      // } else {
      //   // Create standard deviation matrix and reset cooefficient for next periodic iteration
      //   m_stdDevs =
      //       VecBuilder.fill(
      //           VisionConstants.LINEAR_STD_DEV_M * m_stdDevCoeff,
      //           VisionConstants.LINEAR_STD_DEV_M * m_stdDevCoeff,
      //           VisionConstants.ANGULAR_STD_DEV_RAD * m_stdDevCoeff);
      //   m_stdDevCoeff = 0.0;
      //   // Use pose generated from the camera that saw an AprilTag and clear pose list
      //   m_consumer.accept(m_estimatedPoses.get(0), m_inputs[0].timestampSec, m_stdDevs);
      //   m_estimatedPoses.clear();
    }

    // Update pose estimator from limelight
    if (RobotStateConstants.getMode() != RobotStateConstants.Mode.SIM) {
      var estimatedPose = m_inputs[0].limelightPose;
      if (estimatedPose == null) return;
      System.out.println(estimatedPose.toString());
      m_consumer.accept(estimatedPose, Timer.getFPGATimestamp(), m_stdDevs);
      System.out.println("Applied pose estimation");
    }
  }

  /**
   * @param index Camera index.
   * @return PhotonPipelineResult containing latest data calculated by PhotonVision.
   */
  public PhotonPipelineResult getPipelineResult(int index) {
    return m_inputs[index].pipelineResult;
  }

  /**
   * Retrieves the latest pipeline and checks if an AprilTag is seen to determine the ID returned.
   *
   * @param index Camera index.
   * @return ID of AprilTag currently seen, -1 if none seen.
   */
  public int getTagID(int index) {
    var result = this.getPipelineResult(index);
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
