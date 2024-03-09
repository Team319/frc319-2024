// Copyright 2021-2023 FRC 6328
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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    BUSTER,

    /** Running a real robot with a tank drive. */
    TANK,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class WristConstants {
        public static class PID {
          public static final double kP = 1.750; //0.096155
          public static final double kI = 0;
          public static final double kD = 0;
  
          public static final int iZone = 0;
          public static final double kFF = 0.0;
        }
  
        public static class Setpoints {
          public static final float top = (float)0.479; //.4
          public static final float shoot = (float)0.3;
          public static final float home = (float)0.0083; //0.025
          public static final float sub =(float)0.05;
          public static final float podium = (float)0.19; //.2
          public static final float amp =(float)0.475;
          public static final float bottom = (float)0.0083; 
        }
  
        public static class SoftLimits {
          public static final float forwardSoftLimit = Setpoints.top;
          public static final float reverseSoftLimit = Setpoints.bottom;
        }
  
        public static class Currents {
          public static final int currentMax = 20;
          public static final int currentThreshold = 0;
        }
      }

      
}
