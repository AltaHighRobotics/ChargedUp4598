// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import utilities.ConfigurablePID;
import utilities.PIDConfiguration;
import utilities.SwerveModuleConfig;
import utilities.MathTools;

public class SwerveModuleSub extends SubsystemBase {
  /** Creates a new SwerveModuleSub. */
  private SwerveModuleConfig configuration;

  // Wheel.
  private ConfigurablePID wheelPid;
  private double desiredSpeed = 0.0;
  private WPI_TalonFX wheelMotor;
  private double wheelMotorEncoderOffset = 0.0; // Used for reseting motor encoder.

  // Turn.
  private ConfigurablePID turnPid;
  private PIDConfiguration turnPidConfig;
  private double desiredAngle = 0.0;
  private Spark turnMotor;
  private Encoder turnEncoder;
  private int turnMotorDirection = Constants.TURN_MOTOR_CLOCKWISE;
  private double angleError = 0.0;

  public SwerveModuleSub(SwerveModuleConfig config) {
    configuration = config;

    // Wheel motor.
    wheelMotor = new WPI_TalonFX(config.wheelMotorId);
    wheelMotor.configFactoryDefault();
    wheelMotor.setInverted(config.invertWheelMotor);
    wheelMotor.setNeutralMode(NeutralMode.Brake);
    resetWheelEncoder();

    // Turn motor.
    turnMotor = new Spark(config.turnMotorId);
    turnMotor.setInverted(config.invertTurnMotor);

    // Turn encoder.
    turnEncoder = new Encoder(
      new DigitalInput(config.turnEncoderChannelA),
      new DigitalInput(config.turnEncoderChannelB),
      !config.invertTurnMotor
    );

    turnEncoder.setDistancePerPulse(Constants.SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE);
    resetTurnEncoder();

    // Wheel pid.
    wheelPid = new ConfigurablePID(Constants.SWERVE_MODULE_WHEEL_PID);

    // Turn pid.
    turnPid = new ConfigurablePID(Constants.SWERVE_MODULE_TURN_PID);
    turnPidConfig = Constants.SWERVE_MODULE_TURN_PID;
  }

  public void setWheelMotor(double speed) {
    wheelMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopWheelMotor() {
    wheelMotor.neutralOutput();
  }

  public void setTurnMotor(double speed) {
    turnMotor.set(speed);
  }

  public void stopTurnMotor() {
    turnMotor.stopMotor();
  }

  public void stop() {
    stopWheelMotor();
    stopTurnMotor();
  }

  public void resetTurnEncoder() {
    turnEncoder.reset();
  }

  public double getAngle() {
    return MathUtil.inputModulus(turnEncoder.getDistance(), 0.0, 360.0);
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public double getDistance() {
    return (wheelMotorEncoderOffset - wheelMotor.getSelectedSensorPosition()) / Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE;
  }

  public void resetWheelEncoder() {
    wheelMotorEncoderOffset = wheelMotor.getSelectedSensorPosition();
  }

  public double getSpeed() {
    return wheelMotor.getSelectedSensorVelocity() / Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE;
  }

  public void setDesiredSpeed(double desiredSpeed) {
    this.desiredSpeed = desiredSpeed;
  }

  public double getSpeedError() {
    return wheelPid.getError();
  }

  public double getAngleError() {
    return angleError;
  }

  public boolean atSpeed() {
    return getSpeedError() == 0.0;
  }

  public boolean atAngle() {
    return getAngleError() == 0.0;
  }
  
  public void run() {
    // Wheel.
    //setWheelMotor(wheelPid.runPID(desiredSpeed, getSpeed()));

    double angleErrorAbs = Math.abs(desiredAngle - getAngle());

    // Set motor direction and proportional gain.
    // Don't run code in if statement if already set to correct direction.
    if (angleErrorAbs <= 180.0 && turnMotorDirection != Constants.TURN_MOTOR_CLOCKWISE) { // Clockwise
      turnMotorDirection = Constants.TURN_MOTOR_CLOCKWISE;
      turnPid.setProportionalGain(turnPidConfig.proportionalGain * turnMotorDirection);
    } else if (angleErrorAbs > 180.0 && turnMotorDirection != Constants.TURN_MOTOR_COUNTERCLOCKWISE) { // Counterclockwise
      turnMotorDirection = Constants.TURN_MOTOR_COUNTERCLOCKWISE;
      turnPid.setProportionalGain(turnPidConfig.proportionalGain * turnMotorDirection);
    }

    // Turn.
    setTurnMotor(turnPid.runPID(desiredAngle, getAngle()));
  }

  @Override
  public void periodic() {
  }
}
