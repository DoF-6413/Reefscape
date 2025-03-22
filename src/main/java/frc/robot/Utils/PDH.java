package frc.robot.Utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.RobotStateConstants;

public class PDH {
  public final PowerDistribution m_pdh;

  public PDH() {
    // Initialize PDH
    m_pdh = new PowerDistribution(RobotStateConstants.PDH_CAN_ID, ModuleType.kRev);

    // Clear sticky faults so only relevant ones appear
    m_pdh.clearStickyFaults();
  }

  /**
   * Change the power status of Switchable Channel 23 on the PDH.
   *
   * @param enable {@code true} to enable power, {@code false} to disable
   */
  public void enableSwitchable(boolean enable) {
    m_pdh.setSwitchableChannel(enable);
  }
}
