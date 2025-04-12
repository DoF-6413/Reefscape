package frc.robot.Utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Periscope.Periscope;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d m_SwerveBase;
  private final MechanismRoot2d m_periscopeRoot;
  private final MechanismLigament2d m_periscopeLigament;
  private final Periscope m_periscope;

  public Mechanisms2d(Periscope m_periscope) {
    m_SwerveBase = new Mechanism2d(.9144, 2.23012);

    this.m_periscope = m_periscope;

    m_periscopeRoot = m_SwerveBase.getRoot("Periscope", 0.0762, 0.04445);
    m_periscopeLigament = m_periscopeRoot.append(new MechanismLigament2d("Periscope", 0.9652, 90));
  }

  @Override
  public void periodic() {
    m_periscopeLigament.setLength(m_periscope.getHeightMeters());
    SmartDashboard.putData("Mechanisms2d", m_SwerveBase);
  }
}
