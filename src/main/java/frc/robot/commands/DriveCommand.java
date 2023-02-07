// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSub;
import utilities.MathTools;
import utilities.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private DriveTrainSub m_driveTrainSub;
  private XboxController m_driveController;

  private double rightStickX;
  private double rightStickY;
  private double leftStickX;
  private double leftStickY;

  private double strafe = 0.0;
  private double speed = 0.0;
  private double rotation = 0.0;

  public DriveCommand(DriveTrainSub driveTrainSub, XboxController driveController) {
    m_driveTrainSub = driveTrainSub;
    m_driveController = driveController;

    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // its debugging time )---:
  private void test() {
    SwerveModule testModule = m_driveTrainSub.getSwerveModuleFromId(2);
    SmartDashboard.putNumber("Angle", testModule.getAngle());

    double deg = MathTools.wrapAngle((m_driveController.getRawAxis(4) + 1) * 180);
    double speed = Math.abs(m_driveController.getRawAxis(1) * 5.11480);

    if (speed < 1) {
      speed = 0.0;
    }

    SmartDashboard.putNumber("joystick", deg);
    SmartDashboard.putNumber("speed joystick", speed);
    SmartDashboard.putNumber("Error", testModule.getSpeedError());

    testModule.setDesiredSpeed(speed);
    testModule.setDesiredAngle(deg);
    //testModule.setWheelMotor(1.0);
    testModule.run();

    SmartDashboard.putNumber("Wheel speed", testModule.getSpeed());
    SmartDashboard.putNumber("Wheel position", testModule.getDistance());
  }

  private void test2() {
    double deg = MathTools.wrapAngle((m_driveController.getRawAxis(4) + 1) * 180);
    double speed = Math.abs(m_driveController.getRawAxis(1) * 0.511480);

    SmartDashboard.putNumber("joystick", deg);
    SmartDashboard.putNumber("speed joystick", speed);

    for (SwerveModule module : m_driveTrainSub.getSwerveModules()) {
      module.setDesiredAngle(deg);
      module.setDesiredSpeed(speed);
    }

    m_driveTrainSub.run();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrainSub.resetAllEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    test2();
    
    /*
    // Get joystick axis.
    rightStickX = m_driveController.getRawAxis(Constants.RIGHT_STICK_X);
    rightStickY = m_driveController.getRawAxis(Constants.RIGHT_STICK_Y);
    leftStickY = m_driveController.getRawAxis(Constants.LEFT_STICK_Y);
    leftStickX = m_driveController.getRawAxis(Constants.LEFT_STICK_X);

    // Apply dead zones to controller.
    if (Math.abs(rightStickX) < Constants.DRIVE_CONTROLLER_RIGHT_DEAD_ZONE) {
      rightStickX = 0.0;
    } if (Math.abs(rightStickY) < Constants.DRIVE_CONTROLLER_RIGHT_DEAD_ZONE) {
      rightStickY = 0.0;
    } if (Math.abs(leftStickX) < Constants.DRIVE_CONTROLLER_LEFT_DEAD_ZONE) {
      leftStickX = 0.0;
    } if (Math.abs(leftStickY) < Constants.DRIVE_CONTROLLER_LEFT_DEAD_ZONE) {
      leftStickY = 0.0;
    }

    // Set strafe, speed, and rotation.
    strafe = leftStickX;
    speed = leftStickY;
    rotation = rightStickX;

    // Set swerve drive.
    m_driveTrainSub.setSwerveDrive(strafe, -speed, rotation);

    // Call run method to run PID loops and other stuff.
    m_driveTrainSub.run();
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
