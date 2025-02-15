package frc.robot.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.Vision;

public class PathPlanner {
  private final Drive m_drive;
  private final Vision m_vision;

  /**
   * Constructs a new PathPlanner instance
   *
   * <p>This creates a new PathPlanner object that builds the autonomous paths and routines, and
   * Pathfinding paths for the robot to run
   *
   * @param drive Drive subsystem
   * @param pose Pose Estimator
   */
  public PathPlanner(Drive drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;

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
        PathPlannerConstants.ROBOT_CONFIG,
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
   * Creates a command that has the robot follow a path to a 2D pose. Uses default path constraints
   * which sets the max linear velocity and acceleration to 3 m/s, the angular velocity to 515.65
   * deg/s and angular acceleration to 262.82 deg/s^2. Also sets the goal end velocity (the linear
   * velocity of the robot at the end of the path) to 0 m/s
   *
   * @param targetPose Pose2d of where the center of the robot should end up
   */
  public Command pathfindToPose(Pose2d targetPose) {
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
