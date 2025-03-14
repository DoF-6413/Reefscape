package frc.robot.Subsystems.Vision;

import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.LimelightHelpers.LimelightResults;
import frc.robot.Utils.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {

  /**
   * Constructs a new {@link VisionIOLimelight} instance.
   *
   * <p>This creates a new {@link VisionIO} object that uses a Limelight 2 for pose estimation
   *
   * @param index Number corresponding to camera that is to be initilized (0 - Front, 1 - Back, 2 -
   *     Limelight)
   */
  public VisionIOLimelight(int index) {
    System.out.println("[Init] Creating VisionIOLimelight " + VisionConstants.CAMERA_NAMES[index]);

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight",
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getX(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getY(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getZ(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getX(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getY(),
        VisionConstants.CAMERA_ROBOT_OFFSETS[index].getRotation().getZ());
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Get raw AprilTag/Fiducial data
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
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
