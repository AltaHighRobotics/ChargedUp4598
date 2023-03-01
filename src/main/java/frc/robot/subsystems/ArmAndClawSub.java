// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import java.util.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import utilities.ConfigurablePID;
import frc.robot.Constants;

public class ArmAndClawSub extends SubsystemBase {
  /** Creates a new ArmAndClawSub. */
  private Solenoid clawPiston1;
  private Solenoid clawPiston2;

  private WPI_TalonFX smallArmMotor;
  private StatorCurrentLimitConfiguration smallArmMotorCurrentLimit;

  private WPI_TalonFX bigArmMotor;
  private StatorCurrentLimitConfiguration bigArmMotorCurrentLimit;

  private ConfigurablePID smallArmPid;
  private ConfigurablePID bigArmPid;

  private double smallArmSetPoint = 0.0;
  private double bigArmSetPoint = 0.0;

  private DigitalInput armLimitSwitch;

  private double smallArmEncoderOffset = 0.0;
  private double bigArmEncoderOffset = 0.0;

  enum PositioningOrders {
    SAME_TIME,
    SMALL_ARM_FIRST,
    BIG_ARM_FIRST
  }

  private PositioningOrders positioningOrder = PositioningOrders.SAME_TIME;

  public class ArmPositioningData {
    public double smallArmSetPoint;
    public double bigArmSetPoint;
    public PositioningOrders positioningOrder;

    public ArmPositioningData(double smallArmSetPoint, double bigArmSetPoint, PositioningOrders positioningOrder) {
      this.smallArmSetPoint = smallArmSetPoint;
      this.bigArmSetPoint = bigArmSetPoint;
      this.positioningOrder = positioningOrder;
    }
  }

  enum ArmPositionOptions {
    NONE,
    GRAB,
    HIGHER,
    LOWER,
    MIDDLE,
    REST
  }

  private ArmPositionOptions lastPositionOption = ArmPositionOptions.NONE;

  Vector<ArmPositioningData> armPositions = new Vector<ArmPositioningData>();

  public ArmAndClawSub() {
    // Solenoids.
    clawPiston1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_1);
    //clawPiston2 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_2);

    // Motors.
    smallArmMotor = new WPI_TalonFX(Constants.SMALL_ARM_MOTOR);
    smallArmMotor.configFactoryDefault();

    bigArmMotor = new WPI_TalonFX(Constants.BIG_ARM_MOTOR);
    bigArmMotor.configFactoryDefault();

    // Config big arm motor.
    bigArmMotorCurrentLimit = new StatorCurrentLimitConfiguration(true, Constants.BIG_ARM_CURRENT_LIMIT, 0.0, 0.0);

    bigArmMotor.configStatorCurrentLimit(bigArmMotorCurrentLimit);
    bigArmMotor.setNeutralMode(NeutralMode.Brake);
    bigArmMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // Config small arm motor.
    smallArmMotorCurrentLimit = new StatorCurrentLimitConfiguration(true, Constants.SMALL_ARM_CURRENT_LIMIT, 0.0, 0.0);

    smallArmMotor.configStatorCurrentLimit(smallArmMotorCurrentLimit);
    smallArmMotor.setNeutralMode(NeutralMode.Brake);
    smallArmMotor.setInverted(TalonFXInvertType.Clockwise);

    // build team issue!!!

    // Limit swich
    armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

    // PID PID PID
    smallArmPid = new ConfigurablePID(Constants.SMALL_ARM_PID);
    bigArmPid = new ConfigurablePID(Constants.BIG_ARM_PID);

    resetBigArmEncoder();
    resetSmallArmEncoder();
  }

  public boolean getLimitSwitchValue() {
    return !armLimitSwitch.get();
  }

  private void clearPositions() {
    armPositions.clear();
  }

  public void stopMotors() {
    stopBigArmMotor();
    stopSmallArmMotor();
  }

  public void stopBigArmMotor() {
    bigArmMotor.neutralOutput();
    bigArmPid.resetValues();
  }

  public void stopSmallArmMotor() {
    smallArmMotor.neutralOutput();
    smallArmPid.resetValues();
  }

  public void setSmallArmMotor(double power) {
    smallArmMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void setBigArmMotor(double power) {
    if (getLimitSwitchValue() && power < 0) {
      stopBigArmMotor();
      SmartDashboard.putBoolean("Big arm at limit", true);
    } else {
      bigArmMotor.set(TalonFXControlMode.PercentOutput, power);
      SmartDashboard.putBoolean("Big arm at limit", false);
    }
  }

  public void setSmallArmSetPoint(double setpoint) {
    smallArmSetPoint = setpoint;
  }

  public void setBigArmSetPoint(double setpoint) {
    bigArmSetPoint = setpoint;
  }

  public double getSmallArmPosition() {
    return smallArmMotor.getSelectedSensorPosition() * Constants.SMALL_ARM_ENCODER_DISTANCE_PER_PULSE - smallArmEncoderOffset;
  }

  public double getBigArmPosition() {
    return bigArmMotor.getSelectedSensorPosition() * Constants.BIG_ARM_ENCODER_DISTANCE_PER_PULSE - bigArmEncoderOffset;
  }

  public void resetSmallArmEncoder() {
    smallArmEncoderOffset -= getSmallArmPosition();
  }

  public void resetBigArmEncoder() {
    bigArmEncoderOffset += getBigArmPosition();
  }

  public boolean isBigArmAtPosition() {
    return Math.abs(getBigArmPosition() - bigArmSetPoint) <= Constants.BIG_ARM_THRESHOLD;
  }

  public boolean isSmallArmAtPosition() {
    return Math.abs(getSmallArmPosition() - smallArmSetPoint) <= Constants.SMALL_ARM_THRESHOLD;
  }

  public boolean isArmAtPosition() {
    return isBigArmAtPosition() && isSmallArmAtPosition();
  }

  public boolean atLastPosition() {
    return armPositions.isEmpty();
  }

  @Override
  public void periodic() {
    run();
  }

  public void run() {
    updatePositions();
    runArmPids();

    SmartDashboard.putNumber("Big arm setpoint", bigArmSetPoint);
    SmartDashboard.putNumber("Small arm setpoint", smallArmSetPoint);

    SmartDashboard.putBoolean("Limit switch", getLimitSwitchValue());

    SmartDashboard.putNumber("Small arm position", getSmallArmPosition());
    SmartDashboard.putNumber("Big arm position", getBigArmPosition());
    SmartDashboard.putNumber("Arm Positions count", armPositions.size());
  }

  private void runArmPids() {
    boolean bigArmAtPosition, smallArmAtPosition;
    bigArmAtPosition = isBigArmAtPosition();
    smallArmAtPosition = isSmallArmAtPosition();

    SmartDashboard.putBoolean("Small arm at position", smallArmAtPosition);
    SmartDashboard.putBoolean("Big arm at position", bigArmAtPosition);

    switch (positioningOrder) {
      case SAME_TIME:
        setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
        setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));

        SmartDashboard.putString("Arm positioning order", "same time");
        break;
      case SMALL_ARM_FIRST:
        setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));

        if (smallArmAtPosition) {
          setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));
        } else {
          stopBigArmMotor();
        }

        SmartDashboard.putString("Arm positioning order", "small arm first");
        break;
      case BIG_ARM_FIRST:
        setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));

        if (bigArmAtPosition) {
          setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
        } else {
          stopSmallArmMotor();
        }

        SmartDashboard.putString("Arm positioning order", "big arm first");
        break;
      default:
        SmartDashboard.putString("Arm positioning order", "other");
        break;
    }  
  }

  // Update the arm setpoints based on data in armPositions.
  private void updatePositions() {
    if (armPositions.isEmpty()) {
      return;
    }

    // Check if arm is at position.
    boolean armAtPosition = isArmAtPosition();

    // Is at position.
    if (armAtPosition) {
      applyNextArmPosition();
    }
  }

  private void applyNextArmPosition() {
    ArmPositioningData positioningData;

    if (armPositions.isEmpty()) {
      return;
    }
    
    // Get end position.
    positioningData = armPositions.firstElement();

    // Apply.
    this.bigArmSetPoint = positioningData.bigArmSetPoint;
    this.smallArmSetPoint = positioningData.smallArmSetPoint;
    this.positioningOrder = positioningData.positioningOrder;

    // Remove from vector.
    armPositions.remove(positioningData);
  }

  public void setArmPositions(Vector<ArmPositioningData> armPositions) {
    this.armPositions = armPositions;
    applyNextArmPosition();
  }

  public void clawOpen() {
    clawPiston1.set(true);
    SmartDashboard.putBoolean("Claw open", true);
  }

  public void clawClose() {
    clawPiston1.set(false);
    SmartDashboard.putBoolean("Claw open", false);
  }

  public void toggleClaw() {
    clawPiston1.set(!clawPiston1.get());
    SmartDashboard.putBoolean("Claw open", clawPiston1.get());
  }

  public void armGrab() {
    if (lastPositionOption == ArmPositionOptions.GRAB) {
      return;
    }

    setBigArmSetPoint(-37849.0);
    setSmallArmSetPoint(5202.0);
    SmartDashboard.putString("Arm position", "grab");
    positioningOrder = PositioningOrders.BIG_ARM_FIRST;
    clearPositions();

    lastPositionOption = ArmPositionOptions.GRAB;
  }

  public void armLower() {
    if (lastPositionOption == ArmPositionOptions.LOWER) {
      return;
    }

    setBigArmSetPoint(16709.0);
    setSmallArmSetPoint(116009.0);
    SmartDashboard.putString("Arm position", "Lower");
    positioningOrder = PositioningOrders.SAME_TIME;
    clearPositions();

    lastPositionOption = ArmPositionOptions.LOWER;
  }

  public void armMiddle() {
    if (lastPositionOption == ArmPositionOptions.MIDDLE) {
      return;
    }

    setBigArmSetPoint(-34057.0);
    setSmallArmSetPoint(77681.0);

    SmartDashboard.putString("Arm position", "middle");
    positioningOrder = PositioningOrders.SAME_TIME;

    lastPositionOption = ArmPositionOptions.MIDDLE;
  }

  public void armHigher() {
    if (lastPositionOption == ArmPositionOptions.HIGHER) {
      return;
    }

    //setBigArmSetPoint(23309.0);
    //setSmallArmSetPoint(84157.0);

    SmartDashboard.putString("Arm position", "higher");
    //positioningOrder = PositioningOrders.SAME_TIME;
    //clearPositions();

    Vector<ArmPositioningData> newArmPositions = new Vector<ArmPositioningData>();
    newArmPositions.add(new ArmPositioningData(84157.0, -37849.0, PositioningOrders.SAME_TIME));
    newArmPositions.add(new ArmPositioningData(84157.0, 23309.0, PositioningOrders.SAME_TIME));
    setArmPositions(newArmPositions);

    lastPositionOption = ArmPositionOptions.HIGHER;
  }

  public void armRest() {
    if (lastPositionOption == ArmPositionOptions.REST) {
      return;
    }

    setBigArmSetPoint(8972.0);
    setSmallArmSetPoint(16821.0);
    SmartDashboard.putString("Arm position", "rest");
    positioningOrder = PositioningOrders.SAME_TIME;
    clearPositions();

    lastPositionOption = ArmPositionOptions.REST;
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
