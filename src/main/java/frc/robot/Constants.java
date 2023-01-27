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
    // Swerve module.
    public static final double SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE = 0.0;
    public static final double SWERVE_MODULE_WHEEL_CIRCUMFERENCE = 0.0;
    public static final double SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE = 0.0;

    public static final PIDConfiguration SWERVE_MODULE_WHEEL_PID = new PIDConfiguration(0, 0, 0, 0.4, 0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);
    public static final PIDConfiguration SWERVE_MODULE_TURN_PID = new PIDConfiguration(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 1.0);

    public static final int FRONT_RIGHT_MODULE = 0;
    public static final int FRONT_LEFT_MODULE = 1;
    public static final int BACK_RIGHT_MODULE = 2;
    public static final int BACK_LEFT_MODULE = 3;
    public static final int SWERVE_MODULE_COUNT = 4;

    public static final SwerveModuleConfig []SWERVE_MODULE_CONFIGS = {
        new SwerveModuleConfig(0, 0, 0, 0, false, false), // Front right
        new SwerveModuleConfig(0, 0, 0, 0, false, false), // Front left
        new SwerveModuleConfig(0, 0, 0, 0, false, false), // Back right
        new SwerveModuleConfig(0, 0, 0, 0, false, false) // Back left
    };
    
    public static final double VEHICLE_WHEELBASE = 0.0;
    public static final double VEHICLE_TRACKWIDTH = 0.0;
    public static final double VEHICLE_RADIUS = Math.hypot(VEHICLE_WHEELBASE, VEHICLE_TRACKWIDTH);

    // Controllers.
    public static final int DRIVE_CONTROLLER = 0;
    public static final double DRIVE_CONTROLLER_RIGHT_DEAD_ZONE = 0.0;
    public static final double DRIVE_CONTROLLER_LEFT_DEAD_ZONE = 0.0;

    public static final int RIGHT_STICK_Y = 3;
    public static final int RIGHT_STICK_X = 4;
    public static final int LEFT_STICK_Y = 1;
    public static final int LEFT_STICK_X = 0;

    // Turn motor directions.
    public static final int TURN_MOTOR_CLOCKWISE = 1;
    public static final int TURN_MOTOR_COUNTERCLOCKWISE = -1;
}
