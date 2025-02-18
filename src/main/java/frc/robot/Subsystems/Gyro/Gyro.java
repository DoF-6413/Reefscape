// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gyro extends SubsystemBase {
  private final GyroIO m_io;
  private final GyroIOInputsAutoLogged m_inputs = new GyroIOInputsAutoLogged();

  /**
   * Constructs a new {@link Gyro} instance.
   *
   * <p>This creates a new Gyro {@link SubsystemBase} object with the given IO implementation which
   * determines whether the methods and inputs are initialized with the real, or replay code.
   *
   * @param io {@link GyroIO} implementation of the current robot mode (no simulation mode)
   */
  public Gyro(GyroIO io) {
    System.out.println("[Init] Creating Gyro");
    m_io = io;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Gyro", m_inputs);
  }

  /**
   * The angle of the Gyro is normalized to a range of negative pi to pi.
   *
   * @return {@link Rotation2d} of yaw angle (about the Z Axis) of the robot in radians.
   */
  public Rotation2d getYaw() {
    return m_inputs.yawPositionRad;
  }

  /**
   * @return Angular velocity (about the z-axis) of the robot in radians per second.
   */
  public double getYawAngularVelocity() {
    return m_inputs.yawVelocityRadPerSec;
  }

  /** Resets the robot heading to the front side of the robot, making it the new 0 degree angle. */
  public void zeroYaw() {
    m_io.zeroHeading();
  }

  /**
   * @return Whether or not the Gyro is connected and signals are recieved.
   */
  public boolean isConnected() {
    return m_inputs.connected;
  }
}
