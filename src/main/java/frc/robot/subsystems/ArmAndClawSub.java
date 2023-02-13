// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import frc.robot.Constants;

public class ArmAndClawSub extends SubsystemBase {
  /** Creates a new ArmAndClawSub. */
  private Solenoid clawPiston1;
  private Solenoid clawPiston2;

  private WPI_VictorSPX smallArmMotor;
  private WPI_VictorSPX bigArmMotor;

  public ArmAndClawSub() {
    clawPiston1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_1);
    clawPiston2 = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLAW_PISTON_2);

    smallArmMotor = new WPI_VictorSPX(Constants.SMALL_ARM_MOTOR);
    bigArmMotor = new WPI_VictorSPX(Constants.BIG_ARM_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  }

  public void armLower() {

  }

  public void armMiddle() {

  }

  public void armHigher() {

  }

  public void armRest() {
    
  }
}
