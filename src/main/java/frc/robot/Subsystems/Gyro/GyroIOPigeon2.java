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

/** GyroIO implementation for the real mode of the robot running a Pigeon 2.0. */
public class GyroIOPigeon2 implements GyroIO {
  // IMU
  private final Pigeon2 m_gyro;

  // Pigeon logged signals
  private StatusSignal<Angle> m_yawDeg;
  private StatusSignal<AngularVelocity> m_yawVelocityDegPerSec;

  /**
   * Constructs a new {@link GyroIOPigeon2} instance.
   *
   * <p>This creates a new {@link GyroIO} object that uses the real Pigeon 2.0 Inertial Measurement
   * Unit (IMU) for updating values related to IMU readings.
   */
  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");

    // Initialize Pigeon 2.0 IMU
    m_gyro = new Pigeon2(GyroConstants.CAN_ID, "Drivetrain");

    // Pigeon configuration
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());

    // Optimize CAN bus usage, disable all signals aside from those refreshed in code
    m_gyro.optimizeBusUtilization();

    // Initialize IMU inputs and set update frequency to be every 0.01 seconds
    m_yawDeg = m_gyro.getYaw();
    m_yawDeg.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
    m_yawVelocityDegPerSec = m_gyro.getAngularVelocityZWorld();
    m_yawVelocityDegPerSec.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Update signals and check if they are recieved
    inputs.connected = BaseStatusSignal.refreshAll(m_yawDeg, m_yawVelocityDegPerSec).isOK();
    // Update logged inputs from IMU
    inputs.yawPositionRad =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Units.degreesToRadians(m_yawDeg.getValueAsDouble())
                    + GyroConstants.HEADING_OFFSET_RAD));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(m_gyro.getAngularVelocityZWorld().getValueAsDouble());
  }

  @Override
  public void zeroHeading() {
    m_gyro.reset();
  }
}
