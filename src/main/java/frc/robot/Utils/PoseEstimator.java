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

public class PoseEstimator extends SubsystemBase {
  // Subsystem
  private final Drive m_drive;

  // Pose Estimation objects
  private final SwerveDrivePoseEstimator m_poseEstimator;
  public final Vector<N3> m_statesStandarDeviation = VecBuilder.fill(0.1, 0.1, 0.1);
  private Field2d m_field;
  private double m_timestamp;

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

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            drive.getRotation(),
            drive.getModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update timestamp
    m_timestamp = Timer.getFPGATimestamp();

    // Update robot position based on Module movments and Gyro reading
    m_poseEstimator.updateWithTime(
        m_timestamp, m_drive.getRotation(), m_drive.getModulePositions());

    // Put robot's current position onto field
    m_field.setRobotPose(getCurrentPose2d());
  }

  /**
   * @return The current 2D position of the robot on the field
   */
  public Pose2d getCurrentPose2d() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current position of the robot
   *
   * @param pose 2D position to set robot to
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_drive.getRotation(), m_drive.getModulePositions(), pose);
  }

  /**
   * @return Current yaw rotation of the robot
   */
  public Rotation2d getRotation() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }
}
