package frc.robot.Utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Periscope.Periscope;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class Mechanisms2d {
    private final Mechanism2d m_SwerveBase;
    private final MechanismRoot2d m_periscopeRoot;
    private final MechanismLigament2d m_periscopeLigament;
    private final Periscope m_periscope;


    
    public Mechanisms2d(Periscope m_periscope) {
        m_SwerveBase = new Mechanism2d(0.9017, 0.127);
        m_periscopeRoot = m_SwerveBase.getRoot("Periscope", 0.0, 0.0);
    }
}
