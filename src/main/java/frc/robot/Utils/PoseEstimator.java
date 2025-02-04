package frc.robot.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimator extends SubsystemBase {
  // Subsystem
  private final Drive m_drive;

  // Swerve Pose Estimation objects
  private final SwerveDrivePoseEstimator m_swervePoseEstimator;
  private double m_timestamp;
  private double m_prevTimestamp;

  // Field objects
  private Field2d m_field;
  private final AprilTagFieldLayout m_aprilTagFieldLayout;

  // Vision Pose Estimation Objects
  private final PhotonPoseEstimator m_visionPoseEstimatorFront;
  private final PhotonPoseEstimator m_visionPoseEstimatorBack;
  private final PhotonCamera m_cameraFront;
  private final PhotonCamera m_cameraBack;
  public final Vector<N3> m_visionStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private boolean m_enableVision = true;

  // PhotonVision result objects
  private PhotonPipelineResult m_tempPipelineResult;
  private PhotonTrackedTarget m_frontTarget;
  private PhotonTrackedTarget m_backTarget;
  private boolean m_hasTargetsFront = false;
  private boolean m_hasTargetsBack = false;
  private int m_fiducialIDFront = 0;
  private int m_fiducialIDBack = 0;
  private double m_poseAmbiguityFront = 0;
  private double m_poseAmbiguityBack = 0;

  /**
   * This constructs a new PoseEstimator instance
   *
   * <p>This creates a new Pose Estimator object that takes the encoder values from each Swerve
   * Module and the Gyro reading to create a 2D posisiton for the robot on the field.
   *
   * @param drive Drive subsystem
   */
  public PoseEstimator(Drive drive) {
    m_drive = drive;

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            drive.getRotation(),
            drive.getModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));

    m_aprilTagFieldLayout =
        new AprilTagFieldLayout(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldLength(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldWidth());

    m_cameraFront = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
    m_cameraBack = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);

    m_visionPoseEstimatorFront =
        new PhotonPoseEstimator(
            m_aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            VisionConstants.FRONT_CAMERA_ROBOT_OFFSET);
    m_visionPoseEstimatorBack =
        new PhotonPoseEstimator(
            m_aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            VisionConstants.BACK_CAMERA_ROBOT_OFFSET);
  }

  @Override
  public void periodic() {
    // Update timestamp
    m_timestamp = Timer.getFPGATimestamp();

    // Update robot position based on Module movments and Gyro reading
    m_swervePoseEstimator.updateWithTime(
        m_timestamp, m_drive.getRotation(), m_drive.getModulePositions());

    // Put robot's current position onto field
    m_field.setRobotPose(getCurrentPose2d());

    if (m_enableVision) {
      // Saves pipeline results from Front camera if present
      m_tempPipelineResult = m_cameraFront.getLatestResult();
      m_hasTargetsFront = m_tempPipelineResult.hasTargets();
      Optional<EstimatedRobotPose> frontPose =
          m_visionPoseEstimatorFront.update(m_tempPipelineResult);
      if (m_hasTargetsFront) {
        m_frontTarget = m_tempPipelineResult.getBestTarget();
        m_fiducialIDFront = m_frontTarget.getFiducialId();
        m_poseAmbiguityFront = m_frontTarget.getPoseAmbiguity();
      }

      // Saves pipeline results from Back camera if present
      m_tempPipelineResult = m_cameraBack.getLatestResult();
      m_hasTargetsBack = m_tempPipelineResult.hasTargets();
      Optional<EstimatedRobotPose> backPose =
          m_visionPoseEstimatorBack.update(m_tempPipelineResult);
      if (m_hasTargetsBack) {
        m_backTarget = m_tempPipelineResult.getBestTarget();
        m_fiducialIDBack = m_backTarget.getFiducialId();
        m_poseAmbiguityBack = m_backTarget.getPoseAmbiguity();
      }

      // Log if the cameras see an AprilTag
      Logger.recordOutput("Vision/Front/HasTarget", m_hasTargetsFront);
      Logger.recordOutput("Vision/Back/HasTarget", m_hasTargetsBack);

      if (!m_hasTargetsFront && !m_hasTargetsBack) {
        // If no AprilTags are seen, immediently end
        return;

      } else if (m_hasTargetsFront && m_hasTargetsBack) {
        // If both cameras see an AprilTag, average their estimated positions and add that
        // measurement to the Swerve Pose Estimator
        if (m_prevTimestamp != m_timestamp) {
          m_prevTimestamp = m_timestamp;

          if (frontPose.isPresent()
              && backPose.isPresent()
              && m_poseAmbiguityFront < 0.2
              && m_poseAmbiguityFront >= 0.0
              && m_poseAmbiguityBack < 0.2
              && m_poseAmbiguityBack >= 0.0
              && m_fiducialIDFront >= 1
              && m_fiducialIDFront <= 22
              && m_fiducialIDBack >= 1
              && m_fiducialIDBack <= 22) {
            Logger.recordOutput(
                "Vision/Front/EstimatedPose", frontPose.get().estimatedPose.toPose2d());
            Logger.recordOutput(
                "Vision/Back/EstimatedPose", backPose.get().estimatedPose.toPose2d());
            m_swervePoseEstimator.addVisionMeasurement(
                averageVisionPoses(
                    frontPose.get().estimatedPose.toPose2d(),
                    backPose.get().estimatedPose.toPose2d()),
                m_timestamp,
                m_visionStandardDeviations);
          }
        }

      } else if (m_hasTargetsFront) {
        // Update Swerve Pose Estimator based on Front camera
        if (m_prevTimestamp != m_timestamp) {
          m_prevTimestamp = m_timestamp;

          if (frontPose.isPresent()
              && m_poseAmbiguityFront < 0.2
              && m_poseAmbiguityFront >= 0.0
              && m_fiducialIDFront >= 1
              && m_fiducialIDFront <= 22) {
            Logger.recordOutput(
                "Vision/Front/EstimatedPose", frontPose.get().estimatedPose.toPose2d());
            m_swervePoseEstimator.addVisionMeasurement(
                frontPose.get().estimatedPose.toPose2d(), m_timestamp, m_visionStandardDeviations);
          }
        }

      } else {
        // Update Swerve Pose Estimator based on Back camera
        if (m_prevTimestamp != m_timestamp) {
          m_prevTimestamp = m_timestamp;

          if (backPose.isPresent()
              && m_poseAmbiguityBack < 0.2
              && m_poseAmbiguityBack >= 0.0
              && m_fiducialIDBack >= 1
              && m_fiducialIDBack <= 22) {
            Logger.recordOutput(
                "Vision/Back/EstimatedPose", backPose.get().estimatedPose.toPose2d());
            m_swervePoseEstimator.addVisionMeasurement(
                backPose.get().estimatedPose.toPose2d(), m_timestamp, m_visionStandardDeviations);
          }
        }
      }
    }
  }

  /**
   * @return The current 2D position of the robot on the field
   */
  public Pose2d getCurrentPose2d() {
    return m_swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current position of the robot
   *
   * @param pose 2D position to set robot to
   */
  public void resetPose(Pose2d pose) {
    m_swervePoseEstimator.resetPosition(m_drive.getRotation(), m_drive.getModulePositions(), pose);
  }

  /**
   * @return Current yaw rotation of the robot
   */
  public Rotation2d getRotation() {
    return m_swervePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Toggles the use of Vision/Cameras to update the robot's position
   *
   * @param enable True = enable, False = disable
   */
  public void enableVision(boolean enable) {
    this.m_enableVision = enable;
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
}
