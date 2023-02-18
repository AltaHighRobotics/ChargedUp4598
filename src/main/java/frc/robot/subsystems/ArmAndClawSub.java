// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import utilities.ConfigurablePID;
import frc.robot.Constants;

public class ArmAndClawSub extends SubsystemBase {
  /** Creates a new ArmAndClawSub. */
  private Solenoid clawPiston1;
  private Solenoid clawPiston2;

  private WPI_TalonFX smallArmMotor;
  private WPI_TalonFX bigArmMotor;

  private ConfigurablePID smallArmPid;
  private ConfigurablePID bigArmPid;

  private double smallArmSetPoint = 0.0;
  private double bigArmSetPoint = 0.0;

  public ArmAndClawSub() {
    //Solenoids.
    clawPiston1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_1);
    clawPiston2 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_2);

    //Motors.
    smallArmMotor = new WPI_TalonFX(Constants.SMALL_ARM_MOTOR);
    smallArmMotor.configFactoryDefault();

    bigArmMotor = new WPI_TalonFX(Constants.BIG_ARM_MOTOR);
    bigArmMotor.configFactoryDefault();

    // PID PID PID
    smallArmPid = new ConfigurablePID(Constants.SMALL_ARM_PID);
    bigArmPid = new ConfigurablePID(Constants.BIG_ARM_PID);
  }

  public void stopMotors() {
    smallArmMotor.neutralOutput();
    bigArmMotor.neutralOutput();
  }

  public void setSmallArmMotor(double power) {
    smallArmMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setBigArmMotor(double power) {
    bigArmMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setSmallArmSetPoint(double setpoint) {
    smallArmSetPoint = setpoint;
  }

  public void setBigArmSetPoint(double setpoint) {
    bigArmSetPoint = setpoint;
  }

  public double getSmallArmPosition() {
    return smallArmMotor.getSelectedSensorPosition() * Constants.SMALL_ARM_ENCODER_DISTANCE_PER_PULSE;
  }

  public double getBigArmPosition() {
    return bigArmMotor.getSelectedSensorPosition() * Constants.BIG_ARM_ENCODER_DISTANCE_PER_PULSE;
  }

  @Override
  public void periodic() {
    run();
  }

  public void run() {
    setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
    setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));
  }

  public void clawOpen() {
    clawPiston1.set(true);
    clawPiston2.set(true);
  }

  public void clawClose() {
    clawPiston1.set(false);
    clawPiston2.set(false);
  }

  public void armGrab() {
    setBigArmSetPoint(0.0);
    setSmallArmSetPoint(0.0);
  }

  public void armLower() {
    setBigArmSetPoint(0.0);
    setSmallArmSetPoint(0.0);
  }

  public void armMiddle() {
    setBigArmSetPoint(0.0);
    setSmallArmSetPoint(0.0);
  }

  public void armHigher() {
    setBigArmSetPoint(0.0);
    setSmallArmSetPoint(0.0);
  }

  public void armRest() {
    setBigArmSetPoint(0.0);
    setSmallArmSetPoint(0.0);
  }

  public double getBigArmError() {
    return bigArmPid.getError();
  }

  public double getSmallArmError() {
    return smallArmPid.getError();
  }

  public double getBigArmErrorAbs() {
    return Math.abs(bigArmPid.getError());
  }

  public double getSmallArmErrorAbs() {
    return Math.abs(smallArmPid.getError());
  }
}
