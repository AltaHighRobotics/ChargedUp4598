// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import utilities.PIDConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import utilities.SwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Speed.
    public static final double DRIVE_SPEED = 0.8; //0.25
    public static final double SMALL_ARM_LOWER_SPEED = -0.15;
    public static final double CLAW_UP_SPEED = 0.5;
    public static final double CLAW_DOWN_SPEED = -0.15;
    public static final double CLAW_SUCK_SPEED = 0.75;
    public static final double CLAW_SPIT_SPEED = -0.75;
    
    // Swerve module.
    public static final double SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE = 12.600000000000001; // 42 steps per rotation
    public static final double SWERVE_MODULE_WHEEL_CIRCUMFERENCE = 0.3092112569295754; // Meters
    public static final double SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE = 2.3486098074077995e-05; // 2048 steps per rotation
    // (7/45) / 2048 * 0.3092112569295754 for the number up there ^
    public static final StatorCurrentLimitConfiguration SWERVE_MODULE_WHEEL_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 50.0, 0.0, 0.0);

    //public static final PIDConfiguration SWERVE_MODULE_WHEEL_PID = new PIDConfiguration(0.04, 0.0038, 0.004, 0.0, 0, 0, 0, 0, 0, 0, 0, 0.0, 1.0);
    //public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0.014, 0.0000002, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, -0.15, 0.15);
    public static final PIDConfiguration SWERVE_MODULE_WHEEL_PID = new PIDConfiguration(0.05, 0.0035, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0.0, 1.0);
    public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0.006, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, -0.15, 0.15);

    public static final int FRONT_RIGHT_MODULE = 0;
    public static final int FRONT_LEFT_MODULE = 1;
    public static final int BACK_RIGHT_MODULE = 2;
    public static final int BACK_LEFT_MODULE = 3;
    public static final int SWERVE_MODULE_COUNT = 4;

    public static final SwerveModuleConfig []SWERVE_MODULE_CONFIGS = {
        new SwerveModuleConfig(20, 9, true, false), // Front right
        new SwerveModuleConfig(1, 3, false, false), // Front left
        new SwerveModuleConfig(5, 8, true, false), // Back right
        new SwerveModuleConfig(7, 6, false, false) // Back left
    };
    
    public static final double VEHICLE_WHEELBASE = 1.0;
    public static final double VEHICLE_TRACKWIDTH = 1.0;
    public static final double VEHICLE_RADIUS = Math.hypot(VEHICLE_WHEELBASE, VEHICLE_TRACKWIDTH);

    // Controllers.
    public static final int DRIVE_CONTROLLER = 0;
    public static final double DRIVE_CONTROLLER_RIGHT_DEAD_ZONE = 0.05;
    public static final double DRIVE_CONTROLLER_LEFT_DEAD_ZONE = 0.05;

    public static final int RIGHT_STICK_Y = 3;
    public static final int RIGHT_STICK_X = 4;
    public static final int LEFT_STICK_Y = 1;
    public static final int LEFT_STICK_X = 0;

    public static final int XBOX_A_BUTTON = 1; 
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_Y_BUTTON = 4;

    public static final int XBOX_LEFT_BUMPER = 5;
    public static final int XBOX_RIGHT_BUMPER = 6;

    public static final int BUTTON_BOX = 1;

    public static final int TOGGLE_BUTTON = 12;
    public static final int BOTTOM_LEFT_BUTTON = 11;
    public static final int MIDDLE_LEFT_BUTTON = 10;
    public static final int TOP_LEFT_BUTTON = 9;
    public static final int BOTTOM_CENTER_BUTTON = 8;
    public static final int MIDDLE_CENTER_BUTTON = 7;
    public static final int TOP_CENTER_BUTTON = 6;
    public static final int BOTTOM_RIGHT_BUTTON = 5;
    public static final int MIDDLE_RIGHT_BUTTON = 4;
    public static final int TOP_RIGHT_BUTTON = 3;

    //Pistons.
    public static final int CLAW_PISTON_1 = 0;//Change later.
    public static final int CLAW_PISTON_2 = 0;//Change later.

    // Claw.
    public static final int CLAW_MOTOR_ONE = 16;
    public static final int CLAW_MOTOR_TWO = 15;
    public static final int CLAW_ENCODER_A = 0;
    public static final int CLAW_ENCODER_B = 0;
    public static final int CLAW_MOTOR_TWO_CURRENT_LIMIT = 10;

    public static final double CLAW_ONE_UPPER_LIMIT = -12.0;
    public static final double CLAW_ONE_LOWER_LIMIT = -4853.0;

    public static final PIDConfiguration CLAW_ONE_PID = new PIDConfiguration(-0.0005, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -1.0, 0.25);
    public static final PIDConfiguration CLAW_TWO_PID = new PIDConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);
    
    // Arm.
    public static final int SMALL_ARM_MOTOR = 11;//Change later.
    public static final int BIG_ARM_MOTOR = 14;//Change later.
    public static final double BIG_ARM_ENCODER_DISTANCE_PER_PULSE = 1.0;//Change later.
    public static final double SMALL_ARM_ENCODER_DISTANCE_PER_PULSE = 1.0;//Change later.
    public static final double BIG_ARM_CURRENT_LIMIT = 33.0;
    public static final double SMALL_ARM_CURRENT_LIMIT = 35.0;
    public static final int ARM_LIMIT_SWITCH = 0;
    public static final double BIG_ARM_THRESHOLD = 13922.0; // 5000
    public static final double SMALL_ARM_THRESHOLD = 1900; // 1848
    public static final int COOLER_SOLENOID = 3;

    public static final PIDConfiguration SMALL_ARM_PID = new PIDConfiguration(0.00004, 0.0, 0.0, 0.0, 10.0, 0, 0, 0, 0, 0, 0, -0.4, 0.4);
    public static final PIDConfiguration BIG_ARM_PID = new PIDConfiguration(0.00006, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.5);

    // Limelight.
    public static final PIDConfiguration LIMELIGHT_HORIZONTAL_PID = new PIDConfiguration(0.03, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);
    public static final PIDConfiguration LIMELIGHT_VERTICAL_PID = new PIDConfiguration(0.09, 0.00015, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);
    public static final PIDConfiguration REFLECTIVE_LIMELIGHT_VERTICAL_PID = new PIDConfiguration(1.0, 0.005, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);

    public static final double LIMELIGHT_VERTICAL_SETPOINT = 0.0;
    
    public static final int LIMELIGHT_APRIL_TAG_PIPELINE = 0;
    public static final int LIMELIGHT_REFLECTIVE_TAPE_PIPELINE = 1;

    // Autonomous.
    public static final PIDConfiguration AUTONOMOUS_DISTANCE_PID = new PIDConfiguration(0.08, 0.00035, 0.005, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.25, 0.25);
    public static final PIDConfiguration AUTONOMOUS_BALANCE_PID = new PIDConfiguration(-0.01, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);
    public static final double BALANCE_AUTO_DRIVE_TO_ANGLE = 12.0;
    public static final double BALANCE_AUTO_SPEED = 0.2;

    //old value for vertical setpoint 0.227
    public static final double DRIVE_BACK_VERTICAL_SETPOINT = 0.350;
    //old value for horizontal setpoint -3.89 -0.96
    public static final double DRIVE_BACK_HORIZONTAL_SETPOINT = -7.800;

    //Placing Command
    public static final int STOP_PLACE_COMMAND_BUTTON = XBOX_X_BUTTON;
    public static final long WAIT_TIME = 500;

    public static final double LEFT_VERTICAL_SETPOINT = 0.64;
    public static final double LEFT_HORIZONTAL_SETPOINT = -9;

    public static final double MIDDLE_VERTICAL_SETPOINT = 9.0;
    public static final double MIDDLE_HORIZONTAL_SETPOINT = -7.800;

    public static final double RIGHT_VERTICAL_SETPOINT = 0.64;
    public static final double RIGHT_HORIZONTAL_SETPOINT = -9;

    // Auto alignment.

    // old values: 1.0, 1.0
    public static final double AUTO_ALIGNMENT_VERTICAL_THRESHOLD = 0.4;
    public static final double AUTO_ALIGNMENT_HORIZONTAL_THRESHOLD = 1.0;

    public static final double AUTO_ALIGNMENT_REFLECTIVE_VERTICAL_THRESHOLD = 0.1;
    public static final double AUTO_ALIGNMENT_REFLECTIVE_HORIZONTAL_THRESHOLD = 1;

    //0.227

}
