package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;

public class PathPlanner {
  private final Drive m_drive;
  private final PoseEstimator m_pose;

  private final RobotConfig m_robotConfig;
  private final ModuleConfig m_moduleConfig;

  /**
   * Constructs a new PathPlanner instance
   *
   * <p>This creates a new PathPlanner object that builds the autonomous paths and routines, and
   * Pathfinding paths for the robot to run
   *
   * @param drive Drive subsystem
   * @param pose Pose Estimator
   */
  public PathPlanner(Drive drive, PoseEstimator pose) {
    m_drive = drive;
    m_pose = pose;

    m_moduleConfig =
        new ModuleConfig(
            DriveConstants.WHEEL_RADIUS_M,
            DriveConstants.MAX_LINEAR_SPEED_M_PER_S,
            PathPlannerConstants.WHEEL_FRICTION_COEFF,
            DCMotor.getKrakenX60(1),
            DriveConstants.DRIVE_GEAR_RATIO,
            DriveConstants.CUR_LIM_A,
            1);
    m_robotConfig =
        new RobotConfig(
            RobotStateConstants.ROBOT_WEIGHT_KG,
            1,
            m_moduleConfig,
            DriveConstants.TRACK_WIDTH_M); // TODO: Get MOI of entire robot

    AutoBuilder.configure(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                PathPlannerConstants.TRANSLATION_KP, 0, PathPlannerConstants.TRANSLATION_KD),
            new PIDConstants(
                PathPlannerConstants.ROTATION_KP, 0, PathPlannerConstants.ROTATION_KD)),
        m_robotConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive);
  }

  /**
   * Creates a command that has the robot follow a path to a 2D pose. Uses default path constraints
   * which sets the max linear velocity and acceleration to 3 m/s, the angular velocity to 515.65
   * deg/s and angular acceleration to 262.82 deg/s^2. Also sets the goal end velocity (the linear
   * velocity of the robot at the end of the path) to 0 m/s
   *
   * @param targetPose Pose2d of where the center of the robot should end up
   */
  public Command pathFindToPose(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }
}
