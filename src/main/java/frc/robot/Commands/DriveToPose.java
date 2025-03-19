package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final Drive m_drive;
  private final Supplier<Pose2d> m_targetPose;

  private final ProfiledPIDController m_linearController =
      new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4, 1.5));
  private final ProfiledPIDController m_angularController =
      new ProfiledPIDController(
          4.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S, 2 * Math.PI));

  private Translation2d m_lastSetpointTranslation = Translation2d.kZero;
  private double m_linearErrorAbs = 0.0;
  private double m_angularErrorAbs = 0.0;
  private boolean m_running = false;
  private Supplier<Pose2d> m_robotPose;

  private Supplier<Translation2d> m_linearFF = () -> Translation2d.kZero;
  private DoubleSupplier m_angularFF = () -> 0.0;

  /**
   * A {@link Command} that drives the robot to a specified {@link Pose2d}. This runs based off two
   * trapezoidal {@link ProfiledPIDController} for linear and angular movement.
   *
   * @param drive {@link Drive} subsystem
   * @param target Goal end pose of the robot as a {@link Pose2d}
   */
  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.m_drive = drive;
    this.m_targetPose = target;
    m_robotPose = () -> drive.getCurrentPose2d();

    // Enable continuous input for theta controller
    m_angularController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_robotPose.get();
    ChassisSpeeds fieldVelocity = m_drive.getChassisSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    m_linearController.reset(
        currentPose.getTranslation().getDistance(m_targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    m_targetPose
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_angularController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    m_lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    m_running = true;

    // Get current pose and target pose
    Pose2d currentPose = m_robotPose.get();
    Pose2d targetPose = m_targetPose.get();

    // Calculate linear speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    m_linearErrorAbs = currentDistance;
    m_linearController.reset(
        m_lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        m_linearController.getSetpoint().velocity);
    double linearVelocityScalar =
        m_linearController.getSetpoint().velocity
            + m_linearController.calculate(m_linearErrorAbs, 0.0);
    if (currentDistance < m_linearController.getPositionTolerance()) linearVelocityScalar = 0.0;
    m_lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(
                new Transform2d(m_linearController.getSetpoint().position, 0.0, Rotation2d.kZero))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        m_angularController.getSetpoint().velocity
            + m_angularController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    m_angularErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (m_angularErrorAbs < m_angularController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d linearVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(new Transform2d(linearVelocityScalar, 0.0, Rotation2d.kZero))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = m_linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(m_angularFF.getAsDouble()) * 3.0;
    linearVelocity =
        linearVelocity.interpolate(
            m_linearFF.get().times(DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity,
            m_angularFF.getAsDouble() * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
            thetaS);

    // Command speeds
    m_drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX(),
            linearVelocity.getY(),
            thetaVelocity,
            currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", m_linearController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", m_angularController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              m_lastSetpointTranslation,
              Rotation2d.fromRadians(m_angularController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_running = false;
    // Clear logs
    // Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    // Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return m_running && m_linearController.atGoal() && m_angularController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double linearTolerance, Rotation2d angularTolerance) {
    return m_running
        && Math.abs(m_linearErrorAbs) < linearTolerance
        && Math.abs(m_angularErrorAbs) < angularTolerance.getRadians();
  }
}
