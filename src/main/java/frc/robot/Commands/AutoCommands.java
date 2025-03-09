package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Algae.EndEffector.AEE;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.CoralEndEffector.CEE;
import frc.robot.Subsystems.CoralEndEffector.CEEConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Periscope.Periscope;

public class AutoCommands {
  public static Command pathfindingAuto(
      Drive drive, Periscope periscope, AlgaePivot algaePivot, AEE aee, CEE cee, Funnel funnel) {
    return PathfindingCommands.pathfindToBranch("G", 0.1)
        .alongWith(SuperstructureCommands.positionsToL2Coral(periscope, algaePivot))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(
            Commands.runOnce(() -> cee.setPercentSpeed(CEEConstants.SCORE_PERCENT_SPEED), cee));
  }
}
