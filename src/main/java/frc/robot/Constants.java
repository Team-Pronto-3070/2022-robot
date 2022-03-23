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

    public static final class SHOOTER {
        public static final int TAL_SHOOTER_ID = 5;
        public static final double RAMP_TIME = 0.2;
        public static final double HIGH_RPM = 4500;
        public static final double LOW_RPM = 1500;
        public static final double RPM_TOLERANCE = 250;

        public static final class FEEDFORWARD {
            public static final double S = 0.9 * 0.77912;
            public static final double V = 0.9 * 0.11497;
            public static final double A = 0.9 * 0.015929;
        }

        public static final class PID {
            public static final double P = 0.45;
            public static final double I = 0.00012;
            public static final double D = 55;
        }
    }

    public static final class INDEXER {
        public static final int TAL_INDEXER_ID = 6;
        public static final int INDEXER_SWITCH_PORT = 9;
        public static final double RAMP_TIME = 0.2;
        public static final double DEADZONE = 0.2;
    }

    public static final class INTAKE {
        public static final int TAL_INTAKE_ID = 8;
        public static final int TAL_EXTENDER_ID = 9;

        public static final double FORWARD_SPEED = 0.3;
        public static final double REVERSE_SPEED = -0.5;

        public static final int[] ENCODER_PORTS = new int[] {8, 7, 6, 5}; //a, b, index, absolute
        public static final double ENCODER_DISTANCE_PER_PULSE = 0.003067961576; //units: radians
                                                            //2 * pi / 2048 ppr * (18 / window motor gear)
        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = 1;
        public static final double DOWN_POSITION = -0.754; //units: radians from horizontal
        public static final double UP_POSITION = 1.507; //vertical
        public static final double HORIZONTAL_POSITION_OFFSET = 0.042;
        //up - down = 2.256 radians
        //up = .282 absolute position
        //horizontal = .042 absolute position
        //down = -.078 absolute position
        
        public static final class EXTENDER_PID {
            public static final double P = 5;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final class EXTENDER_FEEDFORWARD {
            public static final double S = 0;
            public static final double G = 0;
            public static final double V = 0;
            public static final double A = 0;
        }
    }

    public static final class CLIMBER {
        public static final int TAL_FRONT_ID = 10;
        public static final int TAL_BACK_ID = 7;
        
        public static final double RAMP_TIME = 0.2;
    }

    public static final class DRIVE {

        public static final int TAL_LF_ID = 3;
        public static final int TAL_LB_ID = 4;
        public static final int TAL_RF_ID = 2;
        public static final int TAL_RB_ID = 1;

        public static final double RAMP_TIME = 0.4;

        public static final double TRACK_WIDTH = 0.66049;


        //gearbox: 10.71:1
        //wheel circumference = pi * 6 in * 2.54 cm / in * 1 m / 100 cm = 0.478778720407
        //circumference / gear ratio = 0.0447038954628
        // ^ / 2048 (sensor units / rotation) = 0.00002182807396

        //meters / encoder tick
        public static final double SENSOR_POSITION_COEFFICIENT = 0.00002182807396;

        // ^ * 10
        public static final double SENSOR_VELOCITY_COEFFICIENT = 0.0002182807396;

        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_CENTRIPETAL_ACCELERATION = 1;

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
            public static final double ks = 0.64112;
            public static final double kv = 2.3403;
            public static final double ka = 0.33124;
        }

    }

    public static final class TELEOP_COMMAND {
        public static final double JOY_STICK_DEADZONE = 0.15;
        public static final double JOY_STICK_OMEGA_DEADZONE = 0.25;

        public static final double VX_COEFFICIENT = 1;
        public static final double OMEGA_COEFFICIENT = 0.3;

        public static final double SLOW_VX_COEFFICIENT = 0.25;
        public static final double SLOW_OMEGA_COEFFICIENT = 0.2;
    }

    public static final class OI {
        public static final int XBOX_PORT = 0;
    }
}
