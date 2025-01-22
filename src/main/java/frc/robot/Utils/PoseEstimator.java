package frc.robot.Utils;

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
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Gyro.Gyro;

public class PoseEstimator extends SubsystemBase {

  // The standard deviation for the measurements

  public final Vector<N3> statesStandarDeviation = VecBuilder.fill(0.1, 0.1, 0.1);
  private double timestamp;

  private final Drive drive;
  private final Gyro gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

  private Field2d field;

  public PoseEstimator(Drive drive, Gyro gyro) {

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    this.drive = drive;
    this.gyro = gyro;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            gyro.getYaw(),
            drive.getModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    timestamp = Timer.getFPGATimestamp();

    field.setRobotPose(getCurrentPose());
    poseEstimator.updateWithTime(timestamp, gyro.getYaw(), drive.getModulePositions());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getYaw(), drive.getModulePositions(), pose);
  }

  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }
}
