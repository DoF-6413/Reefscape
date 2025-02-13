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

/** Constant values for the Vision subsystem. Index 0 refers to the Front camera (scoring side) and index 1 refers to the Back camera (intake side) */
public class VisionConstants {
  public enum CAMERA {
    FRONT(0),
    BACK(1);

    public final int CAMERA_INDEX;

    CAMERA(int value) {
      CAMERA_INDEX = value;
    }
  }

  // Photon Camera names
  public static final String[] CAMERA_NAMES = {"Front", "Back"};

  /** Offsets the back left camera's position to the center of the robot */
  private static final Transform3d FRONT_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, 0, 0));

  /** Offsets the back right camera's position to the center of the robot */
  private static final Transform3d BACK_CAMERA_ROBOT_OFFSET =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-13.5), 0, Units.inchesToMeters(3.5)),
          new Rotation3d(0, Units.degreesToRadians(35), Math.PI));

  /** Transformation of both camera locations from the center of the robot. */
  public static final Transform3d[] CAMERA_ROBOT_OFFSETS = {
    FRONT_CAMERA_ROBOT_OFFSET, BACK_CAMERA_ROBOT_OFFSET
  };

  /** Field setup with the locations of the AprilTags loaded from WPILib JSON files */
  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT =
      new AprilTagFieldLayout(
          AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags(),
          AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldLength(),
          AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getFieldWidth());

  /** Default distnace away from an AprilTag the robot should be when Pathfinding to it */
  public static final double DEFAULT_APRILTAG_DISTANCE_M = Units.inchesToMeters(8);
}
