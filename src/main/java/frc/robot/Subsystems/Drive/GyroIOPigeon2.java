package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** GyroIO implementation for the real mode of the robot running a Pigeon 2.0. */
public class GyroIOPigeon2 implements GyroIO {
  // IMU
  private final Pigeon2 m_gyro;

  // Pigeon logged signals
  private final StatusSignal<Angle> m_yawDeg;
  private final StatusSignal<Angle> m_rollDeg;
  private final StatusSignal<Angle> m_pitchDeg;
  private final StatusSignal<AngularVelocity> m_yawVelocityDegPerSec;

  // PhoenixOdometryThread queues
  private final Queue<Double> m_yawPositionQueue;
  private final Queue<Double> m_yawTimestampQueue;

  /**
   * Constructs a new {@link GyroIOPigeon2} instance.
   *
   * <p>This creates a new {@link GyroIO} object that uses the real Pigeon 2.0 Inertial Measurement
   * Unit (IMU) for updating values related to IMU readings.
   */
  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");

    // Initialize Pigeon 2.0 IMU
    m_gyro = new Pigeon2(DriveConstants.GYRO_CAN_ID, "Drivetrain");

    // Pigeon configuration
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());

    // Optimize CAN bus usage, disable all signals aside from those refreshed in code
    m_gyro.optimizeBusUtilization();

    // Initialize IMU inputs and set update frequency to be every 0.01 seconds
    m_yawDeg = m_gyro.getYaw();
    m_rollDeg = m_gyro.getRoll();
    m_pitchDeg = m_gyro.getPitch();
    m_yawDeg.setUpdateFrequency(DriveConstants.ODOMETRY_UPDATE_FREQUENCY_HZ);
    m_yawVelocityDegPerSec = m_gyro.getAngularVelocityZWorld();
    m_yawVelocityDegPerSec.setUpdateFrequency(DriveConstants.UPDATE_FREQUENCY_HZ);

    // Initialize queues
    m_yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_gyro.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Update signals and check if they are recieved
    inputs.connected =
        BaseStatusSignal.refreshAll(m_yawDeg, m_yawVelocityDegPerSec, m_pitchDeg, m_rollDeg).isOK();
    // Update logged inputs from IMU
    inputs.yawPositionRad =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Units.degreesToRadians(m_yawDeg.getValueAsDouble())
                    + DriveConstants.HEADING_OFFSET_RAD));
    inputs.pitch =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Units.degreesToRadians(m_pitchDeg.getValueAsDouble())
                    + DriveConstants.HEADING_OFFSET_RAD));
    inputs.roll =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Units.degreesToRadians(m_rollDeg.getValueAsDouble())
                    + DriveConstants.HEADING_OFFSET_RAD));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(m_gyro.getAngularVelocityZWorld().getValueAsDouble());

    // Update odometry queues
    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(value))))
            .toArray(Rotation2d[]::new);
    m_yawTimestampQueue.clear();
    m_yawPositionQueue.clear();
  }

  @Override
  public void zeroHeading() {
    m_gyro.reset();
  }
}
