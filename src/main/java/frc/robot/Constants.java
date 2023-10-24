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

  public static final class Elevator {

    public static final double POSITION = 1.0;
    public static final double RATE = 0.025;

    public static final double MAX_PERCENT = .7;

    // Heights
    /** Max height is 21.75 inches (adjusted for overshoot) */
    public static final double maxHeight = 20;

    /** Minimum height is 0 inches */
    public static final double minHeight = 0;

    /** the minimum height the arm will stay out of the way for, inches */
    public static final double ENGAGED_HEIGHT = 1.2;

    /** run the motors at lower power when height is higher then upper or lower then lower. */
    public static final class SlowZone {
      public static final double SLOWZONE_MODIFIER = .25;
      public static final double UPPER_THRESHHOLD = 16;
      public static final double LOWER_THRESHHOLD = 7.5;
    }

    public static final double TOP_LIMIT_SWITCH_TRIGGER_HEIGHT = -19.5;
    public static final double BOTTOM_LIMIT_SWITCH_TRIGGER_HEIGHT = -.5;

    public static final int TICKS = 2048;
    public static final double GEAR_RATIO = 12.75;
    public static final double GEAR_CIRCUMFERENCE = 1.5 * Math.PI;

    public static final class Ports {
      public static final int LEFT_MOTOR = 7;
      public static final int RIGHT_MOTOR = 6;
      public static final int BOTTOM_SWITCH = 9;
      public static final int TOP_SWITCH = 0;
    }
  }

}
