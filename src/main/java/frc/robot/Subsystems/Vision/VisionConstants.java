// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Photon Camera names
  public static final String[] CAMERA_NAMES = {"Front", "Back"};

  public enum CAMERA {
    FRONT(0),
    BACK(1);

    public final int CAMERA_INDEX;

    CAMERA(int value) {
      CAMERA_INDEX = value;
    }
  }

  /** Offsets the back left camera's position to the center of the robot */
  public static final Transform3d FRONT_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, 0, 0)); // TODO: Measure out offsets for both cameras

  /** Offsets the back right camera's position to the center of the robot */
  public static final Transform3d BACK_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, Units.degreesToRadians(35), Math.PI));

  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT =
      new AprilTagFieldLayout(
          AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags(),
          AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldLength(),
          AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getFieldWidth());
}
