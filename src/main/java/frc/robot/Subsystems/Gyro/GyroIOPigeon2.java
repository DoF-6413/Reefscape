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

/** Runs real Pigeon 2.0 Gyroscope */
public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 gyro;

  // Pigeon inputs
  private StatusSignal<Angle> yawDeg;
  private StatusSignal<AngularVelocity> yawVelocityDegPerSec;

  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");
    // Ininitalize Pigeon Gyro
    gyro = new Pigeon2(GyroConstants.CAN_ID, "DriveTrain");

    // Pigeon configuration
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.optimizeBusUtilization();

    // Initialize Gyro inputs
    yawDeg = gyro.getYaw();
    yawVelocityDegPerSec = gyro.getAngularVelocityZWorld();

    // Update Gyro signals every 0.01 seconds
    yawDeg.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
    yawVelocityDegPerSec.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
  }

  /**
   * Updates all Gyro signals (inputs) and check if they are connected, including yaw position and
   * velocity
   *
   * <p>Note: Pigeon2 "getYaw" method returns unconstrained yaw in degrees
   *
   * @param inputs - the GyroIOInputs object to update
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yawDeg, yawVelocityDegPerSec).isOK();
    inputs.yawPositionRad =
        Rotation2d.fromRadians(
            MathUtil.inputModulus(
                Units.degreesToRadians(yawDeg.getValueAsDouble())
                    + GyroConstants.HEADING_OFFSET_RAD,
                0,
                2 * Math.PI));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());
    inputs.rawYawPositionRad =
        Rotation2d.fromRadians(Units.degreesToRadians(yawDeg.getValueAsDouble()));
  }

  @Override
  public void zeroHeading() {
    gyro.reset();
  }
}
