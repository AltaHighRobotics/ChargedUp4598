// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndClawSub;

public class TestArmCommad extends CommandBase {
  /** Creates a new TestArmCommad. */
  private ArmAndClawSub m_armAndClawSub;

  public TestArmCommad(ArmAndClawSub armAndClawSub) {
    m_armAndClawSub = armAndClawSub;

    addRequirements(m_armAndClawSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armAndClawSub.resetSmallArmEncoder();
    m_armAndClawSub.resetBigArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armAndClawSub.setSmallArmMotor(-0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armAndClawSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
