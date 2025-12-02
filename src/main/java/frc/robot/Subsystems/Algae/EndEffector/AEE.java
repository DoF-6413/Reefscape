// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Algae.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;

public class AEE extends SubsystemBase {
  private final AEEIO m_io;
  private final AEEIOInputsAutoLogged m_inputs = new AEEIOInputsAutoLogged();

  /**
   * Constructs a new ALGAE End Effector ({@link AEE}) instance.
   *
   * <p>This creates a new AEE {@link SubsystemBase} object with the given IO implementation which
   * determines whether the methods and inputs are initialized with the real, sim, or replay code.
   *
   * @param io {@link AEEIO} implementation of the current mode of the robot.
   */
  public AEE(AEEIO io) {
    System.out.println("[Init] Creating ALGAE End Effector");

    // Initialize the IO implementation
    m_io = io;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // // Update and log inputs // TODO: test to reduce loop time
    // m_io.updateInputs(m_inputs);
    // Logger.processInputs("AEE", m_inputs);
  }

  /**
   * Sets the idle mode of the AEE motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to enable coast mode.
   */
  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Sets voltage of the AEE motor. The value inputed is clamped between values of -12 to 12.
   *
   * @param volts A value between -12 (full reverse speed) to 12 (full forward speed).
   */
  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  /**
   * Sets the speed of the AEE motor based on a percentage.
   *
   * @param percent A value between -1 (full reverse speed) to 1 (full forward speed).
   */
  public void setPercentSpeed(double percent) {
    m_io.setVoltage(percent * RobotStateConstants.MAX_VOLTAGE);
  }
}
