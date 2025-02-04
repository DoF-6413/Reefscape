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
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;

public class PathPlanner {
    private Drive m_drive;
    private PoseEstimator m_pose;
    
    private RobotConfig m_robotConfig;
    private ModuleConfig m_moduleConfig;

    public PathPlanner(Drive drive, PoseEstimator pose) {
        m_drive = drive;
        m_pose = pose;

        m_moduleConfig = new ModuleConfig(DriveConstants.WHEEL_RADIUS_M, DriveConstants.MAX_LINEAR_SPEED_M_PER_S, 1.0, DCMotor.getKrakenX60(1), DriveConstants.DRIVE_GEAR_RATIO, DriveConstants.CUR_LIM_A, 1); //TODO: get coefficient of friction between wheel and carpet
        m_robotConfig = new RobotConfig(125, 1, m_moduleConfig, DriveConstants.TRACK_WIDTH_M); // TODO: Verify MOI and Mass);


        AutoBuilder.configure(
            pose::getCurrentPose2d,
            pose::resetPose,
            drive::getChassisSpeeds,
            drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                0,0,0),
            new PIDConstants(0,0,0)
            ),
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
        drive
            
        );
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
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.DEFAULT_PATH_CONSTRAINTS, 0);
  }
}
