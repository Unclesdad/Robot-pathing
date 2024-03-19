// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double ROBOT_LENGTH = 0.7;
  public static final double ROBOT_WIDTH = 0.7;

  public static final double ROBOT_RADIUS = Math.sqrt((ROBOT_LENGTH + ROBOT_WIDTH) / 2);

  public final class Field {
    public static final double FIELD_LENGTH = 1654; // in cm
    public static final double FIELD_WIDTH = 821; // also in cm

  }
}
