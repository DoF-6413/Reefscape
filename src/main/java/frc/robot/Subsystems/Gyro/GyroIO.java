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
    // Whether or not the signals from the Gyro are being recieved
    public boolean connected = false;
    // Current yaw angle
    public Rotation2d yawPositionRad = new Rotation2d();
    // Unadjusted yaw angle (no applied offset or modulus)
    public Rotation2d rawYawPositionRad = new Rotation2d();
    // The angular velocity of the yaw
    public double yawVelocityRadPerSec = 0.0;
  }

  /**
   * Peridocially updates the logged inputs for the Gyro.
   *
   * @param inputs Inputs from the auto logger
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Resets the heading to be whereever the front of the robot is facing (front being the opposite
   * side to the battery)
   */
  public default void zeroHeading() {}
}
