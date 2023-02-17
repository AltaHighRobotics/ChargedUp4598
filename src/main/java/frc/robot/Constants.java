// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import utilities.PIDConfiguration;
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
    public static final double DRIVE_SPEED = 0.25; //0.25
    
    // Swerve module.
    public static final double SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE = 12.600000000000001; // 42 steps per rotation
    public static final double SWERVE_MODULE_WHEEL_CIRCUMFERENCE = 0.3092112569295754; // Meters
    public static final double SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE = 2.3486098074077995e-05; // 2048 steps per rotation
    // (7/45) / 2048 * 0.3092112569295754 for the number up there ^

    //public static final PIDConfiguration SWERVE_MODULE_WHEEL_PID = new PIDConfiguration(0.04, 0.0038, 0.004, 0.0, 0, 0, 0, 0, 0, 0, 0, 0.0, 1.0);
    //public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0.014, 0.0000002, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, -0.15, 0.15);
    public static final PIDConfiguration SWERVE_MODULE_WHEEL_PID = new PIDConfiguration(0.05, 0.0035, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0.0, 1.0);
    public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0.008, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, -0.2, 0.2);

    public static final int FRONT_RIGHT_MODULE = 0;
    public static final int FRONT_LEFT_MODULE = 1;
    public static final int BACK_RIGHT_MODULE = 2;
    public static final int BACK_LEFT_MODULE = 3;
    public static final int SWERVE_MODULE_COUNT = 4;

    public static final SwerveModuleConfig []SWERVE_MODULE_CONFIGS = {
        new SwerveModuleConfig(20, 9, false, false), // Front right
        new SwerveModuleConfig(1, 3, true, false), // Front left
        new SwerveModuleConfig(5, 8, false, false), // Back right
        new SwerveModuleConfig(7, 6, true, false) // Back left
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

    //Pistons.
    public static final int CLAW_PISTON_1 = 0;//Change later.
    public static final int CLAW_PISTON_2 = 0;//Change later.
    
    //Arm motors.
    public static final int SMALL_ARM_MOTOR = 0;//Change later.
    public static final int BIG_ARM_MOTOR = 0;//Change later.
    public static final double BIG_ARM_ENCODER_DISTANCE_PER_PULSE = 0.0;

    public static final PIDConfiguration SMALL_ARM_PID = new PIDConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);
    public static final PIDConfiguration BIG_ARM_PID = new PIDConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);

    // Limelight.
    public static final PIDConfiguration LIMELIGHT_HORIZONTAL_PID = new PIDConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);
    public static final PIDConfiguration LIMELIGHT_VERTICAL_PID = new PIDConfiguration(0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);

    public static final double LIMELIGHT_VERTICAL_SETPOINT = 0.0;
}
