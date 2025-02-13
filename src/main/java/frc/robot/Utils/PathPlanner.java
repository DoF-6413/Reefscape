package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision;

public class PathPlanner {
  private final Drive m_drive;
  private final Vision m_vision;

  private final RobotConfig m_robotConfig;
  private final ModuleConfig m_moduleConfig;

  public PathPlanner(Drive drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;

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
            1.43564109968,
            m_moduleConfig,
            DriveConstants.getModuleTranslations()); // TODO: Get MOI of entire robot

    AutoBuilder.configure(
        drive::getCurrentPose2d,
        drive::resetPose,
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

    Pathfinding.setPathfinder(new LocalADStarAK());
  }

  /**
   * Creates a command that drives the robot to the inputed position
   *
   * @param targetPose Pose2d of where the robot should end up
   */
  public Command pathFindToPose(Pose2d targetPose) {
    // The pose to pathfind to
    // The constraints to use while pathfinding
    // The goal end velocity of the robot when reaching the target pose
    if (targetPose.getX() > 18
        || targetPose.getY() > 9
        || targetPose.getX() < 0
        || targetPose.getY() < 0) {
      return new PrintCommand(
          "Invalid Pose: "
              + targetPose.toString()); // Do nothing if target pose is outside the field
    }
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0)
        .alongWith(new PrintCommand(targetPose.toString()));
  }
}
