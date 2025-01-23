// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** This Runs the Gyro for all Modes of the Robot */
public class Gyro extends SubsystemBase {

  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    System.out.println("[Init] Creating Gyro");
    this.io = io;
  }

  /** This method is called once per scheduler run. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
  }

  /**
   * @return returns the Yaw (Z Axis) in Radians (-pi, pi)
   */
  public Rotation2d getYaw() {
    return inputs.yawPositionRad;
  }

  /**
   * @return returns the Yaw (Z Axis) in Radians (-pi, pi) without any offset
   */
  public Rotation2d getRawYaw() {
    return inputs.rawYawPositionRad;
  }

  /**
   * @return the angular velocity of the robot in Radians per sec
   */
  public double getYawAngularVelocity() {
    return inputs.yawVelocityRadPerSec;
  }

  /** Resets the Heading to the Direction the Gyro is Facing */
  public void zeroYaw() {
    io.zeroHeading();
  }

  /**
   * @return Whether or not the gyro is connected
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Calculates and returns a new adjusted yaw angle for the robot.
   *
   * This method takes the current yaw angle of the robot (in radians)
   * and adds the specified adjustment angle (also in radians) to it. 
   * The result is a new `Rotation2d` object representing the adjusted yaw angle.
   *
   * @param adjustedAngle The angle to adjust the yaw by in radians.
   * @return A new `Rotation2d` object representing the adjusted yaw angle.
   */
  public Rotation2d adjustedYaw(double adjustedAngle) {
    return inputs.yawPositionRad.plus(new Rotation2d(adjustedAngle));
  }
}
