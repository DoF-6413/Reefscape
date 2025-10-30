package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.CoralEndEffector.CEE;
import frc.robot.Subsystems.CoralEndEffector.CEEConstants;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelConstants;

public class SuperstructureCommands {

  public static Command setSpeeds(
      CEE cee, Funnel funnel, double ceeSpeed, double funnelSpeed) {
    return Commands.runOnce(
        () -> {
          cee.setPercentSpeed(ceeSpeed);
          funnel.setPercentSpeed(funnelSpeed);
        },
        cee,
        funnel);
  }

  /**
   * Sets the speeds of the Superstructure flywheels based on the current objective.
   *
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the speeds to score the current objective.
   */
  public static Command score(CEE cee, Funnel funnel) {
    return SuperstructureCommands.setSpeeds(
        cee, funnel, CEEConstants.SCORE_PERCENT_SPEED, 0);
  }

  private static class SuperstructureState {
    /** Percent speed of the CEE */
    public static double CEESpeed = 0.0;
    /** Percent speed of the Funnel */
    public static double funnelSpeed = 0.0;
    /** Objective of the Superstructure to determine the mechanisms' setpoints */
    public static Objective currentObjective;

    /**
     * Positions and speeds of the Superstructure based on the {@link Objective}. Positions include
     * Periscope height and ALGAE Pivot angle.
     *
     * @param objective Objective to determine mechanism positions and speeds.
     */
    public static void objective(Objective objective) {
      currentObjective = objective;
      switch (currentObjective) {
        case L1:
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          break;

        case L2_CORAL:
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          break;

        case L2_ALGAE:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        case L3_CORAL:
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          break;

        case L3_ALGAE:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        case L4:
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          break;

        case CORAL_INTAKE:
          funnelSpeed = FunnelConstants.INTAKE_PERCENT_SPEED;
          CEESpeed = CEEConstants.INTAKE_PERCENT_SPEED;
          break;

        case ALGAE_GROUND:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        case NET:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        case PROCESSOR:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        case ZERO:
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          break;

        default:
          new RuntimeException("Invalid Objective");
          break;
      }
    }

    /** Determines the setpoints of each mechanism on the Superstructure */
    public enum Objective {
      L1,
      L2_CORAL,
      L2_ALGAE,
      L3_CORAL,
      L3_ALGAE,
      L4,
      CORAL_INTAKE,
      ALGAE_GROUND,
      NET,
      PROCESSOR,
      ZERO
    }
  }
}
