package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
   * Sets the position of the Periscope height and ALGAE Pivot angle.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param periscopeHeight Height of Periscope in meters.
   * @param pivotAngle Angle of ALGAE Pivot in radians.
   * @return {@link Command} that sets the positions of the Superstructure mechanisms.
   */
  public static Command setPositions(
      Periscope periscope, AlgaePivot algaePivot, double periscopeHeight, double pivotAngle) {
    return Commands.runOnce(
        () -> {
          periscope.setPosition(periscopeHeight);
          algaePivot.setAngle(pivotAngle);
        },
        periscope,
        algaePivot);
  }

  /**
   * Sets the speeds of the flywheels on the Superstructure.
   *
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @param aeeSpeed Percent speed of the AEE.
   * @param ceeSpeed Percent speed of the CEE.
   * @param funnelSpeed Percent speed of the Funnel.
   * @return {@link Command} that sets the speeds of the Superstructure mechanisms.
   */
  public static Command setSpeeds(
      AEE aee, CEE cee, Funnel funnel, double aeeSpeed, double ceeSpeed, double funnelSpeed) {
    return Commands.runOnce(
        () -> {
          aee.setPercentSpeed(aeeSpeed);
          cee.setPercentSpeed(ceeSpeed);
          funnel.setPercentSpeed(funnelSpeed);
        },
        aee,
        cee,
        funnel);
  }

  /**
   * Reset the positions and speeds of the Superstructure mechanisms to their defaults.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that resets the positions and speeds of the Superstructure mechanisms.
   */
  public static Command zero(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    SuperstructureState.objective(SuperstructureState.Objective.ZERO);
    return SuperstructureCommands.setPositions(
            periscope,
            algaePivot,
            SuperstructureState.periscopeHeight,
            SuperstructureState.algaePivotAngle)
        .alongWith(
            SuperstructureCommands.setSpeeds(
                aee,
                cee,
                funnel,
                SuperstructureState.AEESpeed,
                SuperstructureState.CEESpeed,
                SuperstructureState.funnelSpeed));
  }

  /**
   * Sets the speeds of the Superstructure flywheels based on the current objective.
   *
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the speeds to score the current objective.
   */
  public static Command score(AEE aee, CEE cee, Funnel funnel) {
    return SuperstructureCommands.setSpeeds(
        aee, cee, funnel, AEEConstants.SCORE_PERCENT_SPEED, CEEConstants.SCORE_PERCENT_SPEED, 0);
  }

  /* ~~~~~~~~~~~~~~~~~~~~ CORAL ~~~~~~~~~~~~~~~~~~~~~~~~~ */

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score CORAL in L1.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score CORAL in L1.
   */
  public static Command positionsToL1(Periscope periscope, AlgaePivot algaePivot) {
    SuperstructureState.objective(SuperstructureState.Objective.L1);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score CORAL in L2.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score CORAL in L2.
   */
  public static Command positionsToL2Coral(Periscope periscope, AlgaePivot algaePivot) {

    SuperstructureState.objective(SuperstructureState.Objective.L2_CORAL);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score CORAL in L3.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score CORAL in L3.
   */
  public static Command positionsToL3Coral(Periscope periscope, AlgaePivot algaePivot) {
    SuperstructureState.objective(SuperstructureState.Objective.L3_CORAL);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score CORAL in L4.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score CORAL in L4.
   */
  public static Command positionsToL4(Periscope periscope, AlgaePivot algaePivot) {
    SuperstructureState.objective(SuperstructureState.Objective.L4);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to pickup CORAL.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the positions to pickup CORAL.
   */
  public static Command coralIntake(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    SuperstructureState.objective(SuperstructureState.Objective.CORAL_INTAKE);
    return SuperstructureCommands.setPositions(
            periscope,
            algaePivot,
            SuperstructureState.periscopeHeight,
            SuperstructureState.algaePivotAngle)
        .alongWith(
            setSpeeds(
                aee,
                cee,
                funnel,
                SuperstructureState.AEESpeed,
                SuperstructureState.CEESpeed,
                SuperstructureState.funnelSpeed))
        .andThen(
            Commands.waitUntil(() -> cee.isBeamBreakTriggered(CEEConstants.EXIT_BEAM_BREAK_PORT)))
        .andThen(Commands.runOnce(() -> cee.setVoltage(0), cee));
  }

  /** ~~~~~~~~~~~~~~~~~~~~~~~~~ ALGAE ~~~~~~~~~~~~~~~~~~~~~~~~~ */

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score ALGAE in L2.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the positions to score ALGAE in L2.
   */
  public static Command intakeL2Algae(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    SuperstructureState.objective(SuperstructureState.Objective.L2_ALGAE);
    return SuperstructureCommands.setPositions(
            periscope,
            algaePivot,
            SuperstructureState.periscopeHeight,
            SuperstructureState.algaePivotAngle)
        .alongWith(
            SuperstructureCommands.setSpeeds(
                aee,
                cee,
                funnel,
                SuperstructureState.AEESpeed,
                SuperstructureState.CEESpeed,
                SuperstructureState.funnelSpeed));
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score ALGAE in L3.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the positions to score ALGAE in L3.
   */
  public static Command intakeL3Algae(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    SuperstructureState.objective(SuperstructureState.Objective.L3_ALGAE);
    return SuperstructureCommands.setPositions(
            periscope,
            algaePivot,
            SuperstructureState.periscopeHeight,
            SuperstructureState.algaePivotAngle)
        .alongWith(
            SuperstructureCommands.setSpeeds(
                aee,
                cee,
                funnel,
                SuperstructureState.AEESpeed,
                SuperstructureState.CEESpeed,
                SuperstructureState.funnelSpeed));
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score ALGAE in the NET.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score ALGAE in the NET.
   */
  public static Command positionsToNet(Periscope periscope, AlgaePivot algaePivot) {
    SuperstructureState.objective(SuperstructureState.Objective.NET);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to score ALGAE in the
   * PROCESSOR.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @return {@link Command} that sets the positions to score ALGAE in the PROCESSOR.
   */
  public static Command positionsToProcessor(Periscope periscope, AlgaePivot algaePivot) {
    SuperstructureState.objective(SuperstructureState.Objective.PROCESSOR);
    return SuperstructureCommands.setPositions(
        periscope,
        algaePivot,
        SuperstructureState.periscopeHeight,
        SuperstructureState.algaePivotAngle);
  }

  /**
   * Sets the position of the Periscope height and ALGAE Pivot angle to pick up ALGAE from the
   * ground.
   *
   * @param periscope {@link Periscope} subsystem
   * @param algaePivot {@link AlgaePivot} subsystem
   * @param aee {@link AEE} subsystem
   * @param cee {@link CEE} subsystem
   * @param funnel {@link Funnel} subsystem
   * @return {@link Command} that sets the positions to pick up ALGAE from the ground.
   */
  public static Command intakeGroundAlgae(
      Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    SuperstructureState.objective(SuperstructureState.Objective.ALGAE_GROUND);
    return SuperstructureCommands.setPositions(
            periscope,
            algaePivot,
            SuperstructureState.periscopeHeight,
            SuperstructureState.algaePivotAngle)
        .alongWith(
            SuperstructureCommands.setSpeeds(
                aee,
                cee,
                funnel,
                SuperstructureState.AEESpeed,
                SuperstructureState.CEESpeed,
                SuperstructureState.funnelSpeed));
  }

  /**
   * Positions and speeds of mechanisms on the Superstructure.
   *
   * <p>Superstructure includes the {@link Periscope}, {@link AlgaePivot}, {@link AEE}, {@link CEE},
   * and {@link Funnel}.
   */
  private static class SuperstructureState {
    /** Height setpoint of the Periscope in meters */
    public static double periscopeHeight = PeriscopeConstants.MIN_HEIGHT_M;
    /** Angle of the ALGAE Pivot in radians */
    public static double algaePivotAngle = AlgaePivotConstants.DEFAULT_ANGLE_RAD;
    /** Percent speed of the AEE */
    public static double AEESpeed = 0.0;
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
          periscopeHeight = PeriscopeConstants.L1_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          AEESpeed = 0.0;
          break;

        case L2_CORAL:
          periscopeHeight = PeriscopeConstants.L2_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          AEESpeed = 0.0;
          break;

        case L2_ALGAE:
          periscopeHeight = PeriscopeConstants.L2_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.REEF_ALGAE_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = AEEConstants.INTAKE_PERCENT_SPEED;
          break;

        case L3_CORAL:
          periscopeHeight = PeriscopeConstants.L3_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          AEESpeed = 0.0;
          break;

        case L3_ALGAE:
          periscopeHeight = PeriscopeConstants.L3_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.REEF_ALGAE_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = AEEConstants.SCORE_PERCENT_SPEED;
          break;

        case L4:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = CEEConstants.SCORE_PERCENT_SPEED;
          AEESpeed = 0.0;
          break;

        case CORAL_INTAKE:
          periscopeHeight = PeriscopeConstants.CORAL_STATION_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = FunnelConstants.INTAKE_PERCENT_SPEED;
          CEESpeed = CEEConstants.INTAKE_PERCENT_SPEED;
          AEESpeed = 0.0;
          break;

        case ALGAE_GROUND:
          periscopeHeight = PeriscopeConstants.MIN_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.GROUND_ALGAE_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = AEEConstants.INTAKE_PERCENT_SPEED;
          break;

        case NET:
          periscopeHeight = PeriscopeConstants.L4_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = AEEConstants.SCORE_PERCENT_SPEED;
          break;

        case PROCESSOR:
          periscopeHeight = PeriscopeConstants.PROCESSOR_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.MAX_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = AEEConstants.SCORE_PERCENT_SPEED;
          break;

        case ZERO:
          periscopeHeight = PeriscopeConstants.MIN_HEIGHT_M;
          algaePivotAngle = AlgaePivotConstants.DEFAULT_ANGLE_RAD;
          funnelSpeed = 0.0;
          CEESpeed = 0.0;
          AEESpeed = 0.0;
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

    /**
     * Whether or not the Periscope and ALGAE Pivot are at their setpoints.
     *
     * @param periscope {@link Periscope} subsystem
     * @param algaePivot {@link AlgaePivot} subsystem
     * @return {@code true} if both at setpoints, {@code false} if not.
     */
    public static boolean atGoal(Periscope periscope, AlgaePivot algaePivot) {
      // return periscope.
      return periscope.atSetpointHeight() && algaePivot.atSetpointAngle();
    }
  }
}
