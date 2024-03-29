// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ArmAndClawSub.ArmPositionOptions;
import utilities.MathTools;
import utilities.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmAndClawSub;
import edu.wpi.first.wpilibj.DriverStation;

import java.lang.Math;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private DriveTrainSub m_driveTrainSub;
  private XboxController m_driveController;
  private ArmAndClawSub m_armAndClawSub;

  private double rightStickX;
  private double rightStickY;
  private double leftStickX;
  private double leftStickY;

  private double strafe = 0.0;
  private double speed = 0.0;
  private double rotation = 0.0;

  public DriveCommand(DriveTrainSub driveTrainSub, XboxController driveController, ArmAndClawSub armAndClawSub) {
    m_driveTrainSub = driveTrainSub;
    m_driveController = driveController;
    m_armAndClawSub = armAndClawSub;

    m_driveTrainSub.resetAllEncoders();
    m_driveTrainSub.resetGyro();

    addRequirements(m_driveTrainSub, m_armAndClawSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // its debugging time )---:
  private void test() {
    SwerveModule testModule = m_driveTrainSub.getSwerveModuleFromId(Constants.FRONT_RIGHT_MODULE);
    double deg = m_driveController.getRawAxis(Constants.LEFT_STICK_X) * 360;

    testModule.setDesiredAngle(deg);

    testModule.run();

    SmartDashboard.putNumber("joystick", deg);
    SmartDashboard.putNumber("Test angle", testModule.getTurnEncoderPosition());
  }

  private void test2() {
    double deg = MathTools.wrapAngle((m_driveController.getRawAxis(4) + 1) * 180);
    double speed = Math.abs(m_driveController.getRawAxis(1) * 0.4);

    SmartDashboard.putNumber("joystick", deg);
    SmartDashboard.putNumber("speed joystick", speed);

    for (SwerveModule module : m_driveTrainSub.getSwerveModules()) {
      module.setDesiredAngle(deg);
      module.setWheelMotor(speed);
    }

    m_driveTrainSub.run();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // For debugin (--:
    //m_driveTrainSub.getSwerveModuleFromId(Constants.FRONT_RIGHT_MODULE).resetTurnEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*
    test();

    if (true) {
      return;
    }
    */
    
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

    SmartDashboard.putNumber("desired speed", speed * Constants.DRIVE_SPEED);
    SmartDashboard.putNumber(
      "Module 0 speed", 
      m_driveTrainSub.getSwerveModuleFromId(0).getSpeed()
    );

    double speedLimit;

    if (m_armAndClawSub.getLastPositionOption() == ArmPositionOptions.REST) {
      speedLimit = Constants.REST_DRIVE_SPEED;
    } else {
      speedLimit = Constants.DRIVE_SPEED;
    }

    SmartDashboard.putNumber("Speed limit", speedLimit);

    // Set swerve drive.
    m_driveTrainSub.setSwerveDriveWithLimit(
      Math.pow(strafe, 2.0) * Math.signum(strafe), 
      -Math.pow(speed, 2.0) * Math.signum(speed), 
      Math.pow(rotation, 2.0) * Math.signum(rotation) * Constants.TURN_SPEED,
      true, 
      speedLimit
    );

    // Call run method to run PID loops and other stuff.
    m_driveTrainSub.run();

    // Claw claw dfijfdkjdfkjldf

    // Don't run while auto.
    if (!DriverStation.isTeleop()) {
      return;
    }

    //System.out.println("hihi");

    if (m_driveController.getAButton()) {
      m_armAndClawSub.setClawTwoMotor(Constants.CLAW_SUCK_SPEED);
    } else if (m_driveController.getBButton()) {
      m_armAndClawSub.setClawTwoMotor(Constants.CLAW_SPIT_SPEED);
    } else {
      m_armAndClawSub.setClawTwoMotor(Constants.CLAW_IDLE_SPEED);
    }
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
