package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SuperstructureConstants.OBJECTIVE;
import frc.robot.Subsystems.Algae.EndEffector.AEE;
import frc.robot.Subsystems.Algae.EndEffector.AEEConstants;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivotConstants;
import frc.robot.Subsystems.CoralEndEffector.CEE;
import frc.robot.Subsystems.CoralEndEffector.CEEConstants;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelConstants;
import frc.robot.Subsystems.Periscope.Periscope;
import frc.robot.Subsystems.Periscope.PeriscopeConstants;

public class SuperstructureCommands {

    /**
     * Sets the position of the Periscope height and Algae Pivot angle.
     * 
     * @param periscope {@link Periscope} subsystem
     * @param algaePivot {@link AlgaePivot} subsystem
     * @param periscopeHeight Height of Periscope in meters.
     * @param pivotAngle Angle of ALGAE Pivot in radians.
     * @return {@link Command} that sets the position of the Superstructure mechanisms.
     */
  public static Command superstructureToPosition(
      Periscope periscope, AlgaePivot algaePivot, double periscopeHeight, double pivotAngle) {
    return Commands.run(
        () -> {
          periscope.setPosition(periscopeHeight);
          algaePivot.setAngle(pivotAngle);
        },
        periscope,
        algaePivot);
  }

  /**
   * Sets the velocities of the flywheels on the Superstructure. 
   * 
   * @param AEE {@link AEE} subsystem
   * @param CEE {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param AEEVelocity Velocity of the AEE in radians per second.
   * @param CEEVelocity Velocity of the CEE in radians per second.
   * @param funnelVelocity Velocity of the Funnel in radians per second.
   * @return {@link Command} that sets the velocities of the Superstructure mechanisms.
   */
  public static Command superstructureScore(AEE AEE, CEE CEE, Funnel funnel, double AEEVelocity, double CEEVelocity, double funnelVelocity) {
    return Commands.run(
        ()-> {
            AEE.setVelocity(AEEVelocity);
            CEE.setVelocity(CEEVelocity);
            funnel.setVelocity(funnelVelocity);
        }, AEE, CEE, funnel);
  }

  public static Command superstructureToL1(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee) {
    SuperstructureState.objective(OBJECTIVE.L1);
    return SuperstructureCommands.superstructureToPosition(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle); // TODO: run the CEE for x amount of time after superstructure is at goal
  }

  /**
   * Positions and velocities of mechanisms on the Peris
   *
   * <p>Superstructure includes the {@link Periscope}, {@link AlgaePivot}, {@link AEE}, {@link CEE}, and
   * {@link Funnel}.
   */
  private static class SuperstructureState {
    /** Height setpoint of the Periscope in meters */
    public static double periscopeHeight = PeriscopeConstants.MIN_HEIGHT_M;
    /** Angle of the ALGAE Pivot in radians */
    public static double algaePivotAngle = AlgaePivotConstants.DEFAULT_POSITION_RAD;
    /** Velocity of the AEE in radians per second */
    public static double AEEVelocity = 0.0;
    /** Velocity of the CEE in radians per second */
    public static double CEEVelocity = 0.0;
    /** Velocity of the Funnel in radians per second */
    public static double funnelVelocity = 0.0;

    /**
     * Positions and velocities of the Superstructure based on the {@link OBJECTIVE}. Positions include Periscope height
     * and Algae Pivot angle.
     *
     * @param objective Objective to determine mechanism positions and velocities.
     */
    public static void objective(OBJECTIVE objective) {
      switch (objective) {
        case L1:
          periscopeHeight = PeriscopeConstants.L1_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = CEEConstants.SCORE_SPEED_RAD_PER_SEC;
          AEEVelocity = 0.0;
          break;

        case L2_CORAL:
          periscopeHeight = PeriscopeConstants.L2_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = CEEConstants.SCORE_SPEED_RAD_PER_SEC;
          AEEVelocity = 0.0;
          break;

        case L2_ALGAE:
          periscopeHeight = PeriscopeConstants.L2_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.ALGAE_PICKUP_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = 0.0;
          AEEVelocity = AEEConstants.INTAKE_SPEED_RAD_PER_SEC;
          break;

        case L3_CORAL:
          periscopeHeight = PeriscopeConstants.L3_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = CEEConstants.SCORE_SPEED_RAD_PER_SEC;
          AEEVelocity = 0.0;
          break;

        case L3_ALGAE:
          periscopeHeight = PeriscopeConstants.L3_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.ALGAE_PICKUP_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = 0.0;
          AEEVelocity = AEEConstants.SCORE_SPEED_RAD_PER_SEC;
          break;

        case L4:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = CEEConstants.SCORE_SPEED_RAD_PER_SEC;
          AEEVelocity = 0.0;
          break;

        case PICKUP:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = FunnelConstants.INTAKE_SPEED_RAD_PER_SEC;
          CEEVelocity = 0.0;
          AEEVelocity = 0.0;

        case NET:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = 0.0;
          AEEVelocity = AEEConstants.SCORE_SPEED_RAD_PER_SEC;

        case PROCESSOR:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_POSITION_RAD;
          funnelVelocity = 0.0;
          CEEVelocity = 0.0;
          AEEVelocity = AEEConstants.SCORE_SPEED_RAD_PER_SEC;

        default:
          new RuntimeException("Invalid Objective");
          break;
      }
    }

    /**
     * Whether or not the Periscope and ALGAE Pivot are at their setpoints.
     * 
     * @param periscope {@link Periscope} subsystem
     * @param algaePivot {@link AlgaePivot} subsystem
     * @return {@code true} if both at setpoints, {@code false} if not.
     */
    public static boolean atGoal(Periscope periscope, AlgaePivot algaePivot) {
        // return periscope.
        return false; // TODO: Add atSetpoint functions for the periscope and aee pivot
    }
  }
}
