package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;

public class PathPlanner {
  private final Drive m_drive;
  private final PoseEstimator m_pose;

  private final RobotConfig m_robotConfig;
  private final ModuleConfig m_moduleConfig;

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
            Units.lbsToKilograms(35),
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

  public void periodic() {}

  /**
   * Creates a command that drives the robot to the inputed position
   *
   * @param targetPose Pose2d of where the robot should end up
   */
  public Command pathFindToPose(Pose2d targetPose) {
    // The pose to pathfind to
    // The constraints to use while pathfinding
    // The goal end velocity of the robot when reaching the target pose
    if (targetPose.getX() > 18 || targetPose.getY() > 9) {
      return new PrintCommand(
          targetPose.toString()); // Do nothing if target pose is outside the field
    }
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
        .alongWith(new PrintCommand(targetPose.toString()));
  }
}
