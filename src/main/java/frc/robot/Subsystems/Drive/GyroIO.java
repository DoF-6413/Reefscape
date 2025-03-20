package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO Interface to log the inputs of and create the default methods for the IMU sensor. */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    /** Whether a signal is being recieved by the Pigeon IMU or not */
    public boolean connected = false;
    /** Current yaw angle as a Rotation2d object */
    public Rotation2d yawPositionRad = new Rotation2d();
    /** Angular velocity about the z-axis (yaw) in radians per second */
    public double yawVelocityRadPerSec = 0.0;
    /** Timestamps of signal readings */
    public double[] odometryYawTimestamps = new double[] {};
    /** Queued yaw position readings */
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    public Rotation2d pitch = new Rotation2d();
    public Rotation2d roll = new Rotation2d();
  }

  /**
   * Updates the logged inputs for the Gyro. Must be called periodically.
   *
   * @param inputs Inputs from the auto logger.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Resets the robot heading to the front side of the robot, making it the new 0 degree angle. */
  public default void zeroHeading() {}
}
