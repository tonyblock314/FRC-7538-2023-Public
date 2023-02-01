// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Controller ports
    public static final int DRIVER_XBOX_PORT = 0;

    // Xbox Controller button number template
    public static final int XBOX_LEFT_STICK_X = 0; 
    public static final int XBOX_LEFT_STICK_Y = 1; // Tank drive forward/backward
    public static final int XBOX_LEFT_TRIGGER = 2; // Intake wheel control
    public static final int XBOX_RIGHT_TRIGGER = 3; // Shooter wheel control
    public static final int XBOX_RIGHT_STICK_X = 4; // Tank drive turning
    public static final int XBOX_RIGHT_STICK_Y = 5;

    // Buttons are on a different enumeration, not currently used by our bot though
    public static final int XBOX_A_BUTTON = 1;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_LEFT_BUMPER = 5;
    public static final int XBOX_RIGHT_BUMPER = 6;
    public static final int XBOX_SCREENIE_BUTTON = 7; // aka back button
    public static final int XBOX_THREE_LINES_BUTTON = 8; // aka start button
    public static final int XBOX_LEFT_STICK_BUTTON = 9; 
    public static final int XBOX_RIGHT_STICK_BUTTON = 10;

    // Motor ports
    public static final int LEFT_1_CAN_ID = 1;
    public static final int LEFT_2_CAN_ID = 2;
    public static final int RIGHT_1_CAN_ID = 3;
    public static final int RIGHT_2_CAN_ID = 4;
    public static final int Leader_SparkMax_ID = 1;
    public static final int Follower_SparkMax_ID = 2;
    // public static final int PIGEON_CAN_ID = 9; ;-;
    
    // Wheel speeds
    public static final double FEED_SPEED = 0.7; // Speed at which intake wheel feeds to shooter wheel
    public static final double MAX_INTAKE_SPEED = 0.575;
    public static final double DEFAULT_SHOOT_SPEED = 0.6;

    // Drive train config
    public static final double MAX_DRIVE_SPEED = 0.9; // MAX POWAAAAAAA!!!
    public static final double DRIVE_TRAIN_RATE_LIMIT = 0.6; // Skew limiters begone
    public static final double TURN_FACTOR = 0.6; // Turn influence decreased
    public static final double SCALING_FACTOR = MAX_DRIVE_SPEED / (1 + TURN_FACTOR);

    public static final double RAW_SENSOR_UNITS_TO_DRIVE_METERS = 1.948201497395833e-5; // Roughly 50000 sensor units per meter
    // 2048 raw units per motor rotation
    // 3 rotations of motor per wheel rotation
    // 0.478.. meters per wheel rotation

    public static final double kDriveP = 0.95;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDrivePTol = 0.1;
    public static final double kDriveDTol = 0.1;

    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnPTol = 0;
    public static final double kTurnDTol = 0;

    public static final double kClimbP = 0;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbPTol = 0;
    public static final double kClimbDTol = 0;

    public static final double MAX_CLIMB_SPEED = 0.8;
    public static final double CLIMB_DISTANCE = 0.1;
    public static final double RAW_SENSOR_UNITS_TO_WINCH_METERS = 0;

    // :(
    // public static final double TRACK_WIDTH_METERS = 0.5; // NOT ACTUALLY, NEED TO FIND OUT
    // public static final double FULL_SPEED_IN_METERS = 1; // NOT ACTUALLY, NEED TO FIND OUT
    // public static final double MAX_SPEED_IN_METERS_PER_SECOND = FULL_SPEED_IN_METERS * MAX_DRIVE_SPEED;
    // public static final double MAX_ACCEL_IN_METERS_PER_SECOND = DRIVE_TRAIN_RATE_LIMIT; // Not crucial due to later use of DifferentialDriveVoltageConstraint
    // public static final double kVoltsPos = 0; // NEED TO CONFIGURE WHOLE SECTION
    // public static final double kVoltsVel = 0;
    // public static final double kVoltsAcc = 0;
    // Baseline values for ramsete controller; should work, provided other values are right- however, check for issues during troubleshooting
    // public static final double kRamseteB = 2;
    // public static final double kRamseteZeta = 0.7;
    // public static final double kPDriveVel = 0; // NEEDS TO BE CONFIGURED TO ROBOT 

    // Dead zones - Sets controller inputs to 0 in a certain range around 0 values to account for stick drift / accidental nudges
    public static final double LY_DEADBAND = 0.15; // left stick, y-axis
    public static final double RX_DEADBAND = 0.15; // right stick, x-axis
    public static final double LT_DEADBAND = 0.15; // left trigger
    public static final double RT_DEADBAND = 0.15; // right trigger
}
