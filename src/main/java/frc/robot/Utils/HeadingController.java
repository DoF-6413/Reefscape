package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HeadingControllerConstants;

/** Add your docs here */
public class HeadingController {

  public HeadingController() {
    // Initialize HeadingController
    System.out.println("[Init] Creating HeadingController");
  }

  public double update(Rotation2d setpoint, Rotation2d gyroAngle, double gyroRate) {
    double output =
        setpoint.minus(gyroAngle).getRadians() * HeadingControllerConstants.KP
            + HeadingControllerConstants.KD * gyroRate;
    SmartDashboard.putBoolean(
        "HeadingControllerAtSetpoint",
        gyroAngle.getRadians() <= Units.degreesToRadians(2) + setpoint.getRadians()
            && gyroAngle.getRadians() >= setpoint.getRadians() - Units.degreesToRadians(2));
    return output;
  }
}
