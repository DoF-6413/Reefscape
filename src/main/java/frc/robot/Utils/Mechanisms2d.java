package frc.robot.Utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Periscope.Periscope;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d m_SwerveBase; // area that the robot uses in total + some extra space to look good 
  private final MechanismRoot2d m_periscopeRoot; // position of the periscope in the robot area 
  private final MechanismLigament2d m_periscopeLigament; // the periscope itself( measuremnets and movement)
  private final Periscope m_periscope; // periscope subsystem

  public Mechanisms2d(Periscope m_periscope) {
    m_SwerveBase = new Mechanism2d(.9144, 2.23012); // 3ft by 7.3ft total area

    this.m_periscope = m_periscope;

    m_periscopeRoot = m_SwerveBase.getRoot("Periscope", 0.0762, 0.04445); // position of the periscope in the robot area
    m_periscopeLigament = m_periscopeRoot.append(new MechanismLigament2d("Periscope", 0.9652, 90)); // 90 degrees is going upwards
  }

  @Override
  public void periodic() {
    m_periscopeLigament.setLength(m_periscope.getHeightMeters());
    SmartDashboard.putData("Mechanisms2d", m_SwerveBase); // logging and visualizing the mechanism 
  }
}
