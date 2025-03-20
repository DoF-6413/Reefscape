package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.LimelightResults;
import frc.robot.Utils.LimelightHelpers.RawFiducial;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
  private Supplier<Pose2d> m_robotPose;

  /**
   * Constructs a new {@link VisionIOLimelight} instance.
   *
   * <p>This creates a new {@link VisionIO} object that uses a Limelight 2 for pose estimation
   *
   * @param index Number corresponding to camera that is to be initilized (0 - Front, 1 - Back, 2 -
   *     Limelight)
   */
  public VisionIOLimelight(int index, Supplier<Pose2d> robotPoseSupplier) {
    System.out.println("[Init] Creating VisionIOLimelight " + VisionConstants.CAMERA_NAMES[index]);

    // Configure limelight
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight",
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getX(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getY(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getZ(),
        Units.radiansToDegrees(VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getX()),
        Units.radiansToDegrees(VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getY()),
        Units.radiansToDegrees(VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getZ()));
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");

    // Initialize robot pose
    m_robotPose = robotPoseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Get estimated pose
    LimelightHelpers.SetRobotOrientation(
        "limelight", m_robotPose.get().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megatag2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // Skip updates if no tags are seen
    if (megatag2.tagCount == 0) {
      inputs.hasTargets = false;
      inputs.fiducialID = 0;
      inputs.poseAmbiguity = -1;
      inputs.limelightPose = null;
      return;
    }

    // Update based on current tags seen
    inputs.hasTargets = true;
    inputs.limelightPose = megatag2.pose;
    // Get raw AprilTag/Fiducial data
    RawFiducial[] fiducials = megatag2.rawFiducials;
    for (RawFiducial fiducial : fiducials) {
      inputs.fiducialID = fiducial.id;
      inputs.poseAmbiguity = fiducial.ambiguity;
    }
  }

  @Override
  public LimelightResults getLimeLightResults() {
    return LimelightHelpers.getLatestResults("limelight");
  }
}
