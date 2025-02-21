package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivotConstants;
import frc.robot.Subsystems.Periscope.Periscope;
import frc.robot.Subsystems.Periscope.PeriscopeConstants;

public class AlgaeCommands {
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

  public static Command mechanisms2Net(Periscope periscope, AlgaePivot AEEPivot) {

    return mechanisms2Position(
        periscope, AEEPivot, PeriscopeConstants.NET_HEIGHT_M, AlgaePivotConstants.NET_POSITION_RAD);
  }

  public static Command mechanisms2PickupL2(Periscope periscope, AlgaePivot AEEPivot) {

    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L2_HEIGHT_M,
        AlgaePivotConstants.ALGEA_PICKUP_POSITION_RAD);
  }

  public static Command mechanisms2PickupL3(Periscope periscope, AlgaePivot AEEPivot) {

    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.L3_HEIGHT_M,
        AlgaePivotConstants.ALGEA_PICKUP_POSITION_RAD);
  }

  public static Command mechanisms2Processor(Periscope periscope, AlgaePivot AEEPivot) {

    return mechanisms2Position(
        periscope,
        AEEPivot,
        PeriscopeConstants.PROCESSOR_HEIGHT_M,
        AlgaePivotConstants.ALGEA_PROCESSOR_POSITION_RAD);
  }
}
