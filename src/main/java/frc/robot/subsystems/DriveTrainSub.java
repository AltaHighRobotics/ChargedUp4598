// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import utilities.SwerveModuleConfig;
import utilities.SwerveModule;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;
import java.util.Vector;
import java.lang.Math;
import utilities.AS5600Encoder;
import utilities.MathTools;
import edu.wpi.first.wpilibj.I2C;

// Usefull https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

public class DriveTrainSub extends SubsystemBase {
  /** Creates a new DriveTrainSub. */
  private SwerveModule[] swerveModuleSubs = new SwerveModule[Constants.SWERVE_MODULE_COUNT];
  private AHRS navx;

  public DriveTrainSub() {
    int i;

    // Config swerve modules.
    for (i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i] = new SwerveModule(Constants.SWERVE_MODULE_CONFIGS[i]);
    }

    navx = new AHRS(Port.kMXP);
    resetGyro();
  }

  public void resetGyro() {
    navx.reset();
    navx.zeroYaw();
  }

  public void resetWheelEncoders() {
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i].resetWheelEncoder();
    }
  }

  public void resetTurnEncoders() {
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i].resetTurnEncoder();
    }
  }

  public double getAvgWheelEncoder() {
    double avg = 0.0;

    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      avg += swerveModuleSubs[i].getDistance();
    }

    return avg / 4;
  }

  public void resetAllEncoders() {
    resetWheelEncoders();
    resetTurnEncoders();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public double getHeading() {
    return navx.getCompassHeading();
  }

  public void stopTurn() {
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i].stopTurnMotor();
    }
  }

  public void stopDrive() {
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i].stopWheelMotor();
    }
  }

  public int numberOfMoulesAtSpeed() {
    int count = 0;

    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      count += swerveModuleSubs[i].atSpeed() ? 1 : 0;
    }

    return count;
  }

  public int numberOfMoulesAtAngle() {
    int count = 0;

    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      count += swerveModuleSubs[i].atAngle() ? 1 : 0;
    }

    return count;
  }

  public boolean allToSpeed() {
    return numberOfMoulesAtSpeed() == Constants.SWERVE_MODULE_COUNT;
  }

  public boolean allToAngle() {
    return numberOfMoulesAtAngle() == Constants.SWERVE_MODULE_COUNT;
  }

  // Look in Constants.java for ids.
  public SwerveModule getSwerveModuleFromId(int id) {
    return swerveModuleSubs[id];
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModuleSubs;
  }

  public void stop() {
    stopTurn();
    stopDrive();
  }

  private static double[] normalizeSpeeds(double []speeds) {
    int i;
    double []normalizedSpeeds = speeds.clone();
    double max = normalizedSpeeds[0];

    // Get max.
    for (double v : normalizedSpeeds) {
      max = Math.max(max, v);
    }

    // Doesn't need to be normalized.
    if (max <= 1) {
      return normalizedSpeeds;
    }

    // Normalize.
    for (i = 0; i < normalizedSpeeds.length; i++) {
      normalizedSpeeds[i] /= max;
    }

    return normalizedSpeeds;
  }

  public void run() {

    // Run the swerve module run methods.
    for (int i = 0; i < Constants.SWERVE_MODULE_COUNT; i++) {
      swerveModuleSubs[i].run();

      // Print debug info.
      /*
      SmartDashboard.putString(
        String.format("Swerve Module %d info", i),
        String.format("Speed: %lf, Angle: %lf", swerveModuleSubs[i].getSpeed(), swerveModuleSubs[i].getAngle())
      );
      */
    }

    SmartDashboard.putNumber("Yaw", getYaw());
  }

  // Usefull stuff: https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
  public void setSwerveDrive(double strafe, double speed, double rotation, boolean fieldCentric) {
    int i;
    double x, y, temp;

    x = strafe;
    y = speed;

    // Field centric.
    double yaw, angleCos, angleSin;

    if (fieldCentric) {
      yaw = Math.toRadians(getYaw());
      angleCos = Math.cos(yaw);
      angleSin = Math.sin(yaw);

      temp = y * angleCos + x * angleSin;
      x = -y * angleSin + x * angleCos;
      y = temp;
    }
  
    double a = x - rotation * (Constants.VEHICLE_WHEELBASE / Constants.VEHICLE_RADIUS);
    double b = x + rotation * (Constants.VEHICLE_WHEELBASE / Constants.VEHICLE_RADIUS);
    double c = y - rotation * (Constants.VEHICLE_TRACKWIDTH / Constants.VEHICLE_RADIUS);
    double d = y + rotation * (Constants.VEHICLE_TRACKWIDTH / Constants.VEHICLE_RADIUS);

    // Calculate module speeds.
    double []moduleSpeeds = {
      Math.hypot(b, c),
      Math.hypot(b, d),
      Math.hypot(a, c),
      Math.hypot(a, d)
    };

    // Calculate module angles.
    double []moduleAngles = {
      Math.atan2(b, c),
      Math.atan2(b, d),
      Math.atan2(a, c),
      Math.atan2(a, d)
    };

    // Normalize speeds.
    moduleSpeeds = normalizeSpeeds(moduleSpeeds);

    // Set speed and angle of each module.
    for (i = 0; i < moduleSpeeds.length; i++) {
      // Covert angle unit.
      moduleAngles[i] = MathTools.makeNonNegAngle(Math.toDegrees(moduleAngles[i]));

      swerveModuleSubs[i].setWheelMotor(moduleSpeeds[i] * Constants.DRIVE_SPEED);
      swerveModuleSubs[i].setDesiredAngle(moduleAngles[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
