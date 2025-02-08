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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends SubsystemBase {
  // Subsystem
  private final Drive m_drive;
  private final Vision m_vision;

  // Swerve Pose Estimation objects
  private final SwerveDrivePoseEstimator m_swervePoseEstimator;
  private double m_timestamp;
  private double m_prevTimestamp;

  // Field objects
  private Field2d m_field;

  // Vision Pose Estimation Objects
  public final Vector<N3> m_visionStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1);
  private boolean m_enableVision = true;

  /**
   * This constructs a new PoseEstimator instance
   *
   * <p>This creates a new Pose Estimator object that takes the encoder values from each Swerve
   * Module, the Gyro reading, and optionally the camera readings to create a 2D posisiton for the
   * robot on the field.
   *
   * @param drive Drive subsystem
   */
  public PoseEstimator(Drive drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            drive.getRotation(),
            drive.getModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
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

    // Update Vision Pose Estimation
    if (m_enableVision) {
      var resultFront = m_vision.getPipelineResult(VisionConstants.CAMERA.FRONT.CAMERA_ID);
      var resultBack = m_vision.getPipelineResult(VisionConstants.CAMERA.BACK.CAMERA_ID);
      boolean hasTargetFront = resultFront.hasTargets();
      boolean hasTargetBack = resultBack.hasTargets();

      if (!hasTargetFront && !hasTargetBack) {
        // No targets
        return;

      } else if (hasTargetFront && hasTargetBack) {
        //
        var targetFront = resultFront.getBestTarget();
        var targetBack = resultBack.getBestTarget();

        if (targetFront.getPoseAmbiguity() < 0.2
            && targetFront.getPoseAmbiguity() >= 0.0
            && targetBack.getPoseAmbiguity() < 0.2
            && targetBack.getPoseAmbiguity() >= 0.0
            && targetFront.getFiducialId() >= 1
            && targetFront.getFiducialId() <= 22
            && targetBack.getFiducialId() >= 1
            && targetBack.getFiducialId() <= 22) {
          var poseFront = m_vision.getEstimatedPose(VisionConstants.CAMERA.FRONT.CAMERA_ID);
          var poseBack = m_vision.getEstimatedPose(VisionConstants.CAMERA.BACK.CAMERA_ID);
          Logger.recordOutput("Vision/Front/EstimatedPose", poseFront);
          Logger.recordOutput("Vision/Back/EstimatedPose", poseFront);
          m_swervePoseEstimator.addVisionMeasurement(
              this.averageVisionPoses(poseFront, poseBack), m_timestamp);
        }  
      } else if (hasTargetFront) {
          
      } else {

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

  public static Pose2d toAprilTag() {
    var nwtInstance = NetworkTableInstance.getDefault();
    boolean hasTargetFront =
        nwtInstance.getEntry("/photonvision/Front/hasTarget").getBoolean(false);
    int frontID =
        (int)
            nwtInstance
                .getEntry("/photonvision/Front/rawBytes/targets/fiducialId")
                .getInteger((long) 18.0);
    var atFields =
        new AprilTagFieldLayout(
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldLength(),
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldWidth());
    // var frontResults = m_cameraFront.getAllUnreadResults();
    // var frontResult = frontResults.get(frontResults.size() - 1);
    System.out.println("-==-=-=-==DOING THE THINGY-=-=-=-=-=-=-");
    if (!hasTargetFront) {
      System.out.println("!!!!!!!!!!HAS NO TARGETS!!!!!!!!!!!!!");
      return new Pose2d(100, 100, new Rotation2d());
    }

    // if (hasTargetFront) {
    System.out.println("[][][]HAS FRONT TARGET[][][]");
    var tagPose2d = atFields.getTagPose(frontID).get().toPose2d();
    var inFrontOfTag =
        new Pose2d(
            tagPose2d.getX()
                + (DriveConstants.TRACK_WIDTH_M + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getCos(),
            tagPose2d.getY()
                + (DriveConstants.TRACK_WIDTH_M + Units.inchesToMeters(8))
                    * tagPose2d.getRotation().getSin(),
            tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
    SmartDashboard.putString("Pose In Front of Tag", inFrontOfTag.toString());
    Logger.recordOutput("Vision/PoseInFrontOfTag", inFrontOfTag);
    return inFrontOfTag;
    // }
    //  else {
    //   System.out.println("()()()HAS BACK TARGET()()())");
    //   var tagPose2d =
    //   atFields.getTagPose(m_backTarget.getFiducialId()).get().toPose2d();
    //   var inFrontOfTag =
    //       new Pose2d(
    //           tagPose2d.getX() + Units.inchesToMeters(24) * tagPose2d.getRotation().getCos(),
    //           tagPose2d.getY() + Units.inchesToMeters(24) * tagPose2d.getRotation().getSin(),
    //           tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
    //   SmartDashboard.putString("Pose In Front of Tag", inFrontOfTag.toString());
    //   Logger.recordOutput("Vision/PoseInFrontOfTag", inFrontOfTag);
    //   return inFrontOfTag;
    // }
  }
}
