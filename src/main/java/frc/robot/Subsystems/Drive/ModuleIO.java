package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO Interface to log the inputs of and create the default methods for the Swerve Modules */
public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    // Drive motor
    /** Whether a signal is being recieved by the Drive motor or not */
    public boolean driveIsConnected = false;
    /** Voltage applied to the Drive motor */
    public double driveAppliedVoltage = 0.0;
    /** Current drawn by the Drive motor in amps */
    public double driveCurrentAmps = 0.0;
    /** Temperature of the Drive motor in celsius */
    public double driveTempCelsius = 0.0;
    /** Distance driven by the Module wheel in radians */
    public double drivePositionRad = 0.0;
    /** Velocity of the Module wheel driven by the Drive motor in radians per sec */
    public double driveVelocityRadPerSec = 0.0;

    // Turn motor
    /** Whether a signal is being recieved by the CANcoder or not */
    public boolean absoluteEncoderIsConnected = false;
    /** Voltage applied to the Turn motor */
    public double turnAppliedVoltage = 0.0;
    /** Current drawn by the Turn motor in amps */
    public double turnCurrentAmps = 0.0;
    /** Temperature of the Turn motor in celsius */
    public double turnTempCelsius = 0.0;
    /** Absolute position of the wheel angle in radians (CANcoder) */
    public Rotation2d turnAbsolutePositionRad = new Rotation2d();
    /** Velocity of the Module wheel driven by the Turn motor in radians per sec (CANcoder) */
    public double turnVelocityRadPerSec = 0.0;

    // Odometry queueing
    /** Timestamps of singal readings */
    public double[] odometryTimestamps = new double[] {};
    /** Queued Drive position signals in radians */
    public double[] odometryDrivePositionsRad = new double[] {};
    /** Queued Turn CANcoder position signals in radians */
    public Rotation2d[] odometryAbsTurnPositions = new Rotation2d[] {};
  }

  /**
   * Updates the logged inputs for the Module. Must be called periodically.
   *
   * @param inputs Inputs from the auto logger.
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Sets voltage of the Drive motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public default void setDriveVoltage(double volts) {}

  /**
   * Sets voltage of the Turn motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public default void setTurnVoltage(double volts) {}

  /**
   * Sets the idle mode of the Drive motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public default void setDriveBrakeMode(boolean enable) {}

  /**
   * Sets the idle mode of the Turn motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public default void setTurnBrakeMode(boolean enable) {}

  /**
   * Sets the velocity of the Drive motor using a PID controller.
   *
   * @param velocityRadPerSec Velocity to set Drive motor to in radians per second.
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /**
   * Sets the PID gains for the Drive motor's PID controller.
   *
   * @param kP Proportional gain value.
   * @param kI Integral gain value.
   * @param kD Derivative gain value.
   */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /**
   * Sets the Feedforward gains for the Drive motor's Feedforward.
   *
   * @param kS Static gain value.
   * @param kV Velocity gain value.
   */
  public default void setDriveFF(double kS, double kV) {}
}
