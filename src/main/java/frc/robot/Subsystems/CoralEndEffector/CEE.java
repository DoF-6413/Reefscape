// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CoralEndEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class CEE extends SubsystemBase {
  private final CEEIO m_io;
  private final CEEIOInputsAutoLogged m_inputs = new CEEIOInputsAutoLogged();

  /**
   * Constructs a new CORAL End Effector ({@link CEE}) instance.
   *
   * <p>This creates a new CEE {@link SubsystemBase} object with the given IO implementation which
   * determines whether the methods and inputs are initialized with the real, sim, or replay code.
   *
   * @param io {@link CEEIO} implementation of the current mode of the robot.
   */
  public CEE(CEEIO io) {
    System.out.println("[Init] Creating Coral End Effector");

    // Initialize the IO implementation
    m_io = io;

    SmartDashboard.putBoolean("Sim/BeamBreak_Entrance", false);
    SmartDashboard.putBoolean("Sim/BeamBreak_Exit", false);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Update and log inputs
    m_io.updateInputs(m_inputs);
    Logger.processInputs("CEE", m_inputs);
  }

  /**
   * Sets the idle mode of the CEE motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the CEE motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the speed of the CEE motor based on a percentage.
   *
   * @param percent A value between -1 (full reverse speed) to 1 (full forward speed).
   */
  public void setPercentSpeed(double percent) {
    m_io.setVoltage(percent * RobotStateConstants.MAX_VOLTAGE);
  }

  /**
   * Triggered means that the beam break is broken (an object is in between the sensor).
   *
   * @return {@code true} if the sensor has been triggered, {@code false} if not.
   */
  public boolean isBeamBreakEntranceTriggered() {

    return m_inputs.isBeamBreakEntranceTriggered;
    // return SmartDashboard.getBoolean("Sim/BeamBreak_Entrance", false);
  }

  /**
   * Triggered means that the beam break is broken (an object is in between the sensor).
   *
   * @return {@code true} if either sensor has been triggered, {@code false} if not.
   */
  public boolean isBeamBreakExitTriggered() {
    if (m_inputs.isBeamBreakExitTriggered) System.out.println("Beambreak was triggered");
    return m_inputs.isBeamBreakExitTriggered;
    // return SmartDashboard.getBoolean("Sim/BeamBreak_Exit", false);
  }
}
