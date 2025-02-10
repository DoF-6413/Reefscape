// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class RobotStateConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    /**
     * @return Robot Mode (Real/Sim/Replay)
     */
    public static Mode getMode() {
      if (RobotBase.isReal()) {
        return Mode.REAL;
      } else if (RobotBase.isSimulation()) {
        return Mode.SIM;
      } else {
        return Mode.REPLAY;
      }
    }

    /**
     * @return Alliance from FMS
     */
    public static Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    /** After 500 seconds, the CAN times out */
    public static final int CAN_CONFIG_TIMEOUT_SEC = 500;

    /** Every 20 ms, periodic commands loop */
    public static final double LOOP_PERIODIC_SEC = 0.02;

    /** Max voltage to send to motor */
    public static final double MAX_VOLTAGE = 12;

    /** Weight of the robot with bumpers and battery */
    public static final double ROBOT_WEIGHT_KG = Units.lbsToKilograms(135);
  }

  /** Controller ports */
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int AUX_CONTROLLER = 1;
  }
  /** Heading Controller */
  public static class HeadingControllerConstants {
    public static final double KP = 0.1;
    public static final double KD = 0.1;
  }

  /** Constants for PathPlanner configurations and PathFinding */
  public final class PathPlannerConstants {
    /* Configuration */
    // PID
    public static final double TRANSLATION_KP = 0.0;
    public static final double TRANSLATION_KD = 0.0;
    public static final double ROTATION_KP = 0.125;
    public static final double ROTATION_KD = 0.0;
    /** Coefficient of friction between wheels and the carpet */
    public static final double WHEEL_FRICTION_COEFF = 0.7;

    /* PathFinding */
    /**
     * Max translational and rotational speed and acceleration used for PathPlanner's PathFinding
     */
    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(3, 3, Units.degreesToRadians(515.65), Units.degreesToRadians(262.82));
  }
}
