// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndClawSub;
import frc.robot.Constants;

public class ArmGrabConeGroundCommand extends CommandBase {
  /** Creates a new ArmGrabConeGroundCommand. */
  private ArmAndClawSub m_armAndClawSub;
  public ArmGrabConeGroundCommand(ArmAndClawSub armAndClawSub) {
    m_armAndClawSub = armAndClawSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armAndClawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armAndClawSub.armGrabConeGround();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
