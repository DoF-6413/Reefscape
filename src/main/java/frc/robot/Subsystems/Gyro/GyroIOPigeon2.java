// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** GyroIO implementation for the real mode of the robot */
public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 m_gyro;

  // Pigeon inputs
  private StatusSignal<Angle> m_yawDeg;
  private StatusSignal<AngularVelocity> m_yawVelocityDegPerSec;

  /**
   * Constructs a new GyroIOPigeon2 instance
   *
   * <p>This creates a new GyroIO object that uses the real Pigeon 2.0 IMU sensor for updating
   * values
   */
  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");
    // Ininitalize Pigeon Gyro
    m_gyro = new Pigeon2(GyroConstants.CAN_ID, "DriveTrain");

    // Pigeon configuration
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.optimizeBusUtilization();

    // Initialize Gyro inputs
    m_yawDeg = m_gyro.getYaw();
    m_yawVelocityDegPerSec = m_gyro.getAngularVelocityZWorld();

    // Update Gyro signals every 0.01 seconds
    m_yawDeg.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
    m_yawVelocityDegPerSec.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(m_yawDeg, m_yawVelocityDegPerSec).isOK();
    inputs.yawPositionRad =
        Rotation2d.fromRadians(
            MathUtil.inputModulus(
                Units.degreesToRadians(m_yawDeg.getValueAsDouble())
                    + GyroConstants.HEADING_OFFSET_RAD,
                0,
                2 * Math.PI));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(m_gyro.getAngularVelocityZWorld().getValueAsDouble());
    inputs.rawYawPositionRad =
        Rotation2d.fromRadians(Units.degreesToRadians(m_yawDeg.getValueAsDouble()));
  }
  
  @Override
  public void zeroHeading() {
    m_gyro.reset();
  }
}
