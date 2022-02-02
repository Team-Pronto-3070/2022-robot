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

    public static final class DRIVE {

        public static final int TAL_LF_ID = 4;
        public static final int TAL_LB_ID = 3;
        public static final int TAL_RF_ID = 2;
        public static final int TAL_RB_ID = 1;

        public static final double TRACK_WIDTH = 0.6;

        public static final double MAX_VELOCITY = 3;
        public static final double MAX_ACCELERATION = 3;

        public static final class LPID {
            public static final double P = 1;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class RPID {
            public static final double P = 1;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class FEEDFORWARD {
            public static final double ks = 0;
            public static final double kv = 0;
            public static final double ka = 0;
        }

    }

    public static final class TELEOP_COMMAND {
        public static final double JOY_STICK_DEADZONE = 0.15;
        public static final double JOY_STICK_OMEGA_DEADZONE = 0.25;

        public static final double VX_COEFFICIENT = 1;
        public static final double OMEGA_COEFFICIENT = .5;
    }

    public static final class OI {

        public static final int JOY_PORT = 0;
        public static final int XBOX_PORT = 2;

        public static final Controller CONTROLLER =  Controller.XBOX;

        public static enum Controller {
            XBOX, JOYSTICK 
        }

    }
}
