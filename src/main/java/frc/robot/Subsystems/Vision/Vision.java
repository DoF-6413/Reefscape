// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.DriveConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final VisionConsumer m_consumer;

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
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
    }
    m_consumer = consumer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < m_inputs.length; i++) {
      m_io[i].updateInputs(m_inputs[i]);
      var cam = VisionConstants.CAMERA.values()[i].toString().toLowerCase();
      cam =
          cam.replace(
              cam.charAt(0),
              Character.toUpperCase(
                  cam.charAt(
                      0))); // Now you may be wondering, is all of this really necessary? Can't you
      // just make the names into an array and use the camera index to
      // retrieve the correct name? To that I say that is the coward's way of
      // doing things... TODO: decide if this is necessary :l
      Logger.processInputs("Vision/" + cam, m_inputs[i]);
    }

    var resultFront = this.getPipelineResult(VisionConstants.CAMERA.FRONT.CAMERA_INDEX);
    var resultBack = this.getPipelineResult(VisionConstants.CAMERA.BACK.CAMERA_INDEX);
    boolean hasTargetFront = resultFront.hasTargets();
    boolean hasTargetBack = resultBack.hasTargets();

    if (!hasTargetFront && !hasTargetBack) {
      // No targets
      return;

    } else if (hasTargetFront && hasTargetBack) {
      // Both see target
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
        var poseFront = this.getEstimatedPose(VisionConstants.CAMERA.FRONT.CAMERA_INDEX);
        var poseBack = this.getEstimatedPose(VisionConstants.CAMERA.BACK.CAMERA_INDEX);
        Logger.recordOutput("Vision/Front/EstimatedPose", poseFront);
        Logger.recordOutput("Vision/Back/EstimatedPose", poseBack);
        m_consumer.accept(
            this.averageVisionPoses(poseFront, poseBack), resultFront.getTimestampSeconds());
      }
    } else if (hasTargetFront) {
      // Only Front sees target
      var targetFront = resultFront.getBestTarget();

      if (targetFront.getPoseAmbiguity() < 0.2
          && targetFront.getPoseAmbiguity() >= 0.0
          && targetFront.getFiducialId() >= 1
          && targetFront.getFiducialId() <= 22) {
        var poseFront = this.getEstimatedPose(VisionConstants.CAMERA.FRONT.CAMERA_INDEX);
        Logger.recordOutput("Vision/Front/EstimatedPose", poseFront);
        m_consumer.accept(poseFront, resultFront.getTimestampSeconds());
      } else if (hasTargetBack) {
        // Only Back sees target
        var targetBack = resultBack.getBestTarget();

        if (targetBack.getPoseAmbiguity() < 0.2
            && targetBack.getPoseAmbiguity() >= 0.0
            && targetBack.getFiducialId() >= 1
            && targetBack.getFiducialId() <= 22) {
          var poseBack = this.getEstimatedPose(VisionConstants.CAMERA.BACK.CAMERA_INDEX);
          Logger.recordOutput("Vision/Back/EstimatedPose", poseBack);
          m_consumer.accept(poseBack, resultBack.getTimestampSeconds());
        }
      }
    }
  }

  public PhotonPipelineResult getPipelineResult(int index) {
    return m_inputs[index].pipelineResult;
  }

  public Pose2d getEstimatedPose(int index) {
    return m_inputs[index].estimatedPose;
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
