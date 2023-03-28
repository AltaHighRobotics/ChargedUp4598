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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;
import utilities.ConfigurablePID;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

// I am a chuncky anime girl UWU!!!!

public class ArmAndClawSub extends SubsystemBase {
  /** Creates a new ArmAndClawSub. */
  //private Solenoid clawPiston1;
  
  private WPI_TalonSRX clawMotorOne;
  private CANSparkMax clawMotorTwo;
  private RelativeEncoder clawTwoEncoder;

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

  private ConfigurablePID clawOnePid;
  private ConfigurablePID clawTwoPid;

  private double clawOneSetpoint = 0.0;
  private double clawTwoSetpoint = 0.0;

  enum PositioningOrders {
    SAME_TIME,
    SMALL_ARM_FIRST,
    BIG_ARM_FIRST,
    CLAW_LAST
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
    HIGHER_GRAB,
    HIGHER,
    LOWER,
    MIDDLE,
    REST,
    HIGHER_CONE,
    MIDDLE_CONE
  }

  private ArmPositionOptions lastPositionOption = ArmPositionOptions.NONE;

  Vector<ArmPositioningData> armPositions = new Vector<ArmPositioningData>();

  public ArmAndClawSub() {
    // Solenoids.
    //clawPiston1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_1);
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

    // Claw motors.
    clawMotorOne = new WPI_TalonSRX(Constants.CLAW_MOTOR_ONE);
    clawMotorOne.setNeutralMode(NeutralMode.Brake);

    clawMotorTwo = new CANSparkMax(Constants.CLAW_MOTOR_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotorTwo.setSmartCurrentLimit(Constants.CLAW_MOTOR_TWO_CURRENT_LIMIT);
    clawMotorTwo.setIdleMode(IdleMode.kCoast);

    // Claw encoders.
    clawTwoEncoder = clawMotorTwo.getEncoder();

    resetClawOneEncoder();
    resetClawTwoEncoder();

    // build team issue!!!

    // Limit swich
    armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

    // PID PID PID
    smallArmPid = new ConfigurablePID(Constants.SMALL_ARM_PID);
    bigArmPid = new ConfigurablePID(Constants.BIG_ARM_PID);

    clawOnePid = new ConfigurablePID(Constants.CLAW_ONE_PID);
    clawTwoPid = new ConfigurablePID(Constants.CLAW_TWO_PID);

    resetBigArmEncoder();
    resetSmallArmEncoder();
  }

  public void resetClawOneEncoder() {
    clawMotorOne.setSelectedSensorPosition(0.0);
  }

  public void resetClawTwoEncoder() {
    clawTwoEncoder.setPosition(0.0);
  }

  public void setClawOneMotor(double power) {
    double pos = getClawOnePosition();

    /*if (pos >= Constants.CLAW_ONE_UPPER_LIMIT && power > 0) {
      stopClawOneMotor();
      SmartDashboard.putBoolean("Arm One At Limit", true);
    } else if (pos <= Constants.CLAW_ONE_LOWER_LIMIT && power < 0) {
      stopClawOneMotor();
      SmartDashboard.putBoolean("Arm One At Limit", true);
    } else {*/
      clawMotorOne.set(TalonSRXControlMode.PercentOutput, power);
      SmartDashboard.putBoolean("At Limit", false);
   // }
    SmartDashboard.putNumber("Wrist Position", pos);
  }

  public void setClawTwoMotor(double power) {
    clawMotorTwo.set(power);
  }

  public void stopClawOneMotor() {
    clawMotorOne.neutralOutput();
  }

  public void stopClawTwoMotor() {
    clawMotorTwo.stopMotor();
  }

  public void stopClawMotors() {
    stopClawOneMotor();
    stopClawTwoMotor();
  }

  public double getClawOnePosition() {
    return clawMotorOne.getSelectedSensorPosition();
  }

  public double getClawTwoPosition() {
    return clawTwoEncoder.getPosition();
  }

  public void setClawOneSetpoint(double setpoint) {
    clawOneSetpoint = setpoint;
  }

  public void setClawTwoSetpoint(double setpoint) {
    clawTwoSetpoint = setpoint;
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
    smallArmMotor.setSelectedSensorPosition(0.0);
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

  // Use this instead of 'isArmAtPosition' when checking if arm is at position in subsystems.
  public boolean isAtFinalPosition() {
    return isArmAtPosition() && atLastPosition();
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
    runClawPids();

    SmartDashboard.putNumber("Big arm setpoint", bigArmSetPoint);
    SmartDashboard.putNumber("Small arm setpoint", smallArmSetPoint);

    SmartDashboard.putBoolean("Limit switch", getLimitSwitchValue());

    SmartDashboard.putNumber("Small arm position", getSmallArmPosition());
    SmartDashboard.putNumber("Big arm position", getBigArmPosition());
    SmartDashboard.putNumber("Arm Positions count", armPositions.size());
  }

  private void runClawPids() {
    //setClawOneMotor(clawOnePid.runPID(clawOneSetpoint, getClawOnePosition()));
    //setClawTwoMotor(clawTwoPid.runPID(clawTwoSetpoint, getClawTwoPosition()));

    SmartDashboard.putNumber("Claw one position", getClawOnePosition());
    SmartDashboard.putNumber("Claw one setpoint", clawOneSetpoint);
    SmartDashboard.putNumber("Claw two position", getClawTwoPosition());
  }

  private void runArmPids() {
    boolean bigArmAtPosition, smallArmAtPosition;
    bigArmAtPosition = isBigArmAtPosition();
    smallArmAtPosition = isSmallArmAtPosition();

    SmartDashboard.putBoolean("Small arm at position", smallArmAtPosition);
    SmartDashboard.putBoolean("Big arm at position", bigArmAtPosition);

    /*
    if (1 == 1 && 5 < 6) {
      return;
    }
    */

    switch (positioningOrder) {
      case SAME_TIME:
        setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
        setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));
        setClawOneMotor(clawOnePid.runPID(clawOneSetpoint, getClawOnePosition()));

        SmartDashboard.putString("Arm positioning order", "same time");
        break;
      case SMALL_ARM_FIRST:
        setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
        setClawOneMotor(clawOnePid.runPID(clawOneSetpoint, getClawOnePosition()));

        if (smallArmAtPosition) {
          setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));
          positioningOrder = PositioningOrders.SAME_TIME;
        } else {
          stopBigArmMotor();
        }

        SmartDashboard.putString("Arm positioning order", "small arm first");
        break;
      case BIG_ARM_FIRST:
        setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));
        setClawOneMotor(clawOnePid.runPID(clawOneSetpoint, getClawOnePosition()));

        if (bigArmAtPosition) {
          setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
          positioningOrder = PositioningOrders.SAME_TIME;
        } else {
          stopSmallArmMotor();
        }

        SmartDashboard.putString("Arm positioning order", "big arm first");
        break;
      case CLAW_LAST:
        setSmallArmMotor(smallArmPid.runPID(smallArmSetPoint, getSmallArmPosition()));
        setBigArmMotor(bigArmPid.runPID(bigArmSetPoint, getBigArmPosition()));

        if (bigArmAtPosition && smallArmAtPosition) {
          setClawOneMotor(clawOnePid.runPID(clawOneSetpoint, getClawOnePosition()));
          positioningOrder = PositioningOrders.SAME_TIME;
        } else {
          stopClawOneMotor();
        }

        SmartDashboard.putString("Arm positioning order", "claw last");
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
    //clawPiston1.set(true);
    SmartDashboard.putBoolean("Claw open", true);
  }

  public void clawClose() {
    //clawPiston1.set(false);
    SmartDashboard.putBoolean("Claw open", false);
  }

  public void toggleClaw() {
    //clawPiston1.set(!clawPiston1.get());
    //SmartDashboard.putBoolean("Claw open", clawPiston1.get());
  }

  public void armHigherCone() {
    if (lastPositionOption == ArmPositionOptions.HIGHER_CONE) {
      return;
    }

    //setBigArmSetPoint(28749.000000);
    //setSmallArmSetPoint(-193868.0);
    setClawOneSetpoint(-945.0);

    SmartDashboard.putString("Arm position", "higher cone");
    positioningOrder = PositioningOrders.SAME_TIME;
    clearPositions();

    Vector<ArmPositioningData> newArmPositions = new Vector<ArmPositioningData>();
    newArmPositions.add(new ArmPositioningData(-193868.0, 0.0, PositioningOrders.SAME_TIME));
    newArmPositions.add(new ArmPositioningData(-193868.0, 28749.0, PositioningOrders.SAME_TIME));
    setArmPositions(newArmPositions);

    lastPositionOption = ArmPositionOptions.HIGHER_CONE;
  }

  public void armMiddleCone() {
    if (lastPositionOption == ArmPositionOptions.MIDDLE_CONE) {
      return;
    }

    setBigArmSetPoint(-31588.0);
    setSmallArmSetPoint(-190000.0);//-191836.0
    setClawOneSetpoint(-358.0);
    
    SmartDashboard.putString("Arm position", "middle cone");
    positioningOrder = PositioningOrders.SAME_TIME;
    clearPositions();

    lastPositionOption = ArmPositionOptions.MIDDLE_CONE;
  }

  public void armGrab() {
    if (lastPositionOption == ArmPositionOptions.GRAB) {
      return;
    }

    setBigArmSetPoint(28258.0); //-9630.0
    setSmallArmSetPoint(-60269.0);
    setClawOneSetpoint(-3076.0); //-4342
    SmartDashboard.putString("Arm position", "grab");
    positioningOrder = PositioningOrders.BIG_ARM_FIRST;
    clearPositions();

    lastPositionOption = ArmPositionOptions.GRAB;
  }

  public void armHighGrab(){
    if (lastPositionOption == ArmPositionOptions.HIGHER_GRAB) {
      return;
    }

    setBigArmSetPoint(26587.0);
    setSmallArmSetPoint(-94019.0); // -91019.0
    setClawOneSetpoint(-4953.0); //-4342.0
    SmartDashboard.putString("Arm position", "higher grab");
    positioningOrder = PositioningOrders.CLAW_LAST;
    clearPositions();

    lastPositionOption = ArmPositionOptions.HIGHER_GRAB;
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

    setBigArmSetPoint(-32300.0);
    setSmallArmSetPoint(-205847.0);
    setClawOneSetpoint(-595.0);

    SmartDashboard.putString("Arm position", "middle");
    positioningOrder = PositioningOrders.SAME_TIME;

    lastPositionOption = ArmPositionOptions.MIDDLE;
  }

  public void armHigher() {
    if (lastPositionOption == ArmPositionOptions.HIGHER) {
      return;
    }

    setBigArmSetPoint(28749.0);
    setSmallArmSetPoint(-214637.0);
    setClawOneSetpoint(-928.0);

    SmartDashboard.putString("Arm position", "higher");
    positioningOrder = PositioningOrders.SAME_TIME;
    clearPositions();

    /*
    Vector<ArmPositioningData> newArmPositions = new Vector<ArmPositioningData>();
    newArmPositions.add(new ArmPositioningData(70000.0, -4714.0, PositioningOrders.SAME_TIME));
    newArmPositions.add(new ArmPositioningData(84157.0, 23309.0, PositioningOrders.SAME_TIME));
    setArmPositions(newArmPositions);
    */

    lastPositionOption = ArmPositionOptions.HIGHER;
  }

  public void armRest() {
    if (lastPositionOption == ArmPositionOptions.REST) {
      return;
    }

    setBigArmSetPoint(9099.0);
    setSmallArmSetPoint(-2676.0);
    setClawOneSetpoint(-43.0);
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
