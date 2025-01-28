// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** This Runs the Gyro for all Modes of the Robot */
public class Gyro extends SubsystemBase {

  private final GyroIO m_io;
  private final GyroIOInputsAutoLogged m_inputs = new GyroIOInputsAutoLogged();

  /**
   * Constructs a new Gyro subsystem instance.
   *
   * <p>This constructor creates a new Gyro object that updates the heading of the robot based on an
   * IMU sensor
   *
   * @param io GyroIO implementation of the current robot mode (only real or blank)
   */
  public Gyro(GyroIO io) {
    System.out.println("[Init] Creating Gyro");
    m_io = io;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Gyro", m_inputs);
  }

  /**
   * The angle of the Gyro is normalized to a range of 0 to 2 pi
   *
   * @return Yaw angle (about the Z Axis) of the robot in Radians
   */
  public Rotation2d getYaw() {
    return m_inputs.yawPositionRad;
  }

  /**
   * @return Yaw angle (about the Z Axis) in Radians without any offset
   */
  public Rotation2d getRawYaw() {
    return m_inputs.rawYawPositionRad;
  }

  /**
   * @return Angular velocity of the robot in Radians per sec
   */
  public double getYawAngularVelocity() {
    return m_inputs.yawVelocityRadPerSec;
  }

  /** Resets the Heading to the direction the Gyro is facing (aka the front of the robot) */
  public void zeroYaw() {
    m_io.zeroHeading();
  }

  /**
   * @return Whether or not the Gyro is connected
   */
  public boolean isConnected() {
    return m_inputs.connected;
  }

  /**
   * Calculates and returns a new adjusted yaw angle for the robot.
   *
   * <p>This method takes the current yaw angle of the robot (in radians) and adds the specified
   * adjustment angle (also in radians) to it. The result is a new `Rotation2d` object representing
   * the adjusted yaw angle.
   *
   * @param adjustedAngleRad The angle to adjust the yaw by in radians.
   * @return A new `Rotation2d` object representing the adjusted yaw angle.
   */
  public Rotation2d adjustedYaw(double adjustedAngleRad) {
    return m_inputs.yawPositionRad.plus(new Rotation2d(adjustedAngleRad));
  }
}
