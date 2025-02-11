// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the Gyro in Every Mode */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    /** Whether or not the signals from the Pigeon are being recieved */
    public boolean connected = false;
    /** Current yaw angle as a Rotation2d object */
    public Rotation2d yawPositionRad = new Rotation2d();
    /** Unadjusted yaw angle (no applied offset or normalization) as a Rotation2d object */
    public Rotation2d rawYawPositionRad = new Rotation2d();
    /** Angular velocity about the z-axis (yaw) in radians per second */
    public double yawVelocityRadPerSec = 0.0;
  }

  /**
   * Updates the logged inputs for the Gyro. Must be called periodically
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Resets the robot heading to the direction the Gyro is facing (aka the front of the robot) */
  public default void zeroHeading() {}
}
