package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivotConstants;
import frc.robot.Subsystems.Periscope.Periscope;
import frc.robot.Subsystems.Periscope.PeriscopeConstants;

public class CoralCommands {

  public static Command mechanisms2Position(
      Periscope periscope, AlgaePivot AEEPivot, double periscopeHeight, double pivotAngle) {

    return Commands.run(
        () -> {
          periscope.setPosition(periscopeHeight);
          AEEPivot.setSetpoint(pivotAngle);
        },
        periscope,
        AEEPivot);
  }

  public static Command mechanisms2L1(Periscope periscope, AlgaePivot AEEPivot) {
    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L1_HEIGHT_M,
        AlgaePivotConstants.CORAL_POSITION_RAD);
  }

  public static Command mechanisms2L2(Periscope periscope, AlgaePivot AEEPivot) {
    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L2_HEIGHT_M,
        AlgaePivotConstants.CORAL_POSITION_RAD);
  }

  public static Command mechanisms2L3(Periscope periscope, AlgaePivot AEEPivot) {
    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L3_HEIGHT_M,
        AlgaePivotConstants.CORAL_POSITION_RAD);
  }

  public static Command mechanisms2L4(Periscope periscope, AlgaePivot AEEPivot) {
    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L4_HEIGHT_M,
        AlgaePivotConstants.CORAL_POSITION_RAD);
  }

  public static Command mechanisms2CoralStation(Periscope periscope, AlgaePivot AEEPivot) {
    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.CORAL_STATION_HEIGHT_M,
        AlgaePivotConstants.CORAL_POSITION_RAD); // TODO: may change
  }
}
