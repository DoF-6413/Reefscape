package frc.robot.Utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Algae.Pivot.AlgaePivot;
import frc.robot.Subsystems.Periscope.Periscope;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d m_SwerveBase; // area that the robot uses in total + some extra space to look good 
  private final MechanismRoot2d m_periscopeRoot; // position of the periscope in the robot area 
  private final MechanismRoot2d m_algaePivotRoot; // position of the algae pivot in the robot area
  private final MechanismLigament2d m_periscopeLigament; // the periscope itself( measurements and movement)
  private final MechanismLigament2d m_algaePivotLigament; // the algae pivot itself( measurements and movement)
  private final Periscope m_periscope; // periscope subsystem
  private final AlgaePivot m_algaePivot; // algae pivot subsystem

  public Mechanisms2d(Periscope m_periscope, AlgaePivot m_algaePivot) {
    m_SwerveBase = new Mechanism2d(.9144, 2.23012); // 3ft by 7.3ft total area

    this.m_periscope = m_periscope;
    this.m_algaePivot = m_algaePivot;

    m_periscopeRoot = m_SwerveBase.getRoot("Periscope", 0.0762, 0.04445); // position of the periscope in the robot area
    m_algaePivotRoot = m_SwerveBase.getRoot("AlgaePivot", 0.12,0.9652); // position of the algae pivot in the robot area TODO: measure actual position
    m_periscopeLigament = m_periscopeRoot.append(new MechanismLigament2d("Periscope", 0.9652, 90)); // 90 degrees is going upwards
    m_algaePivotLigament = m_algaePivotRoot.append(new MechanismLigament2d("Algae Pivot", 0.3048, 0));
  }

  @Override
  public void periodic() {
    m_periscopeLigament.setLength(m_periscope.getHeightMeters());
    m_algaePivotLigament.setAngle(m_algaePivot.getAngleRad());
    SmartDashboard.putData("Mechanisms2d", m_SwerveBase); // logging and visualizing the mechanism 
  }
}
