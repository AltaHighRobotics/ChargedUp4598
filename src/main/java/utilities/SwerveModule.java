// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package utilities;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMaxLowLevel;

public class SwerveModule {
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

  private CANSparkMax turnMotor;
  private RelativeEncoder turnEncoder;

  private double angleError = 0.0;

  public SwerveModule(SwerveModuleConfig config) {
    configuration = config;

    // Wheel motor.
    wheelMotor = new WPI_TalonFX(config.wheelMotorId);
    wheelMotor.configFactoryDefault();
    wheelMotor.setInverted(config.invertWheelMotor);
    wheelMotor.setNeutralMode(NeutralMode.Brake);
    resetWheelEncoder();

    // Turn motor.
    turnMotor = new CANSparkMax(config.turnMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    turnMotor.setInverted(config.invertTurnMotor);

    // // Turn encoder.
    turnEncoder = turnMotor.getEncoder();
    turnEncoder.setPositionConversionFactor(Constants.SWERVE_MODULE_TURN_ENCODER_DISTANCE_PER_PULSE);
    //turnEncoder.setInverted(!config.invertTurnMotor);
    resetTurnEncoder();

    // Wheel pid.
    wheelPid = new ConfigurablePID(Constants.SWERVE_MODULE_WHEEL_PID);

    // // Turn pid.
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
    turnEncoder.setPosition(0.0);
  }

  public double getAngle() {
    return MathUtil.inputModulus(
      getTurnEncoderPosition(),
      0.0, 
      360.0
    );
  }

  public double getTurnEncoderPosition() {
    return turnEncoder.getPosition() * (configuration.invertTurnMotor ? -1 : 1);
  }

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle;
    this.desiredAngle = MathTools.getAngleSetPoint(desiredAngle, getAngle());
  }

  public double getDistance() {
    return (wheelMotor.getSelectedSensorPosition() 
    / Constants.SWERVE_MODULE_WHEEL_ENCODER_DISTANCE_PER_PULSE
    * (configuration.invertWheelMotor ? 1 : -1))
    + wheelMotorEncoderOffset;
  }

  public void resetWheelEncoder() {
    wheelMotorEncoderOffset = getDistance();
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

    // Turn.
    setTurnMotor(turnPid.runPID(desiredAngle, getTurnEncoderPosition()));
  }
}
