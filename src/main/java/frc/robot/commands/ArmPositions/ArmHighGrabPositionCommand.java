// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndClawSub;

public class ArmHighGrabPositionCommand extends CommandBase {
  /** Creates a new ArmHighGrabPositionCommand. */
  private ArmAndClawSub m_ArmAndClawSub;
  public ArmHighGrabPositionCommand(ArmAndClawSub armAndClawSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmAndClawSub = armAndClawSub;
    addRequirements(armAndClawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmAndClawSub.armHighGrab();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
