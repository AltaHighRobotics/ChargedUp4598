// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class TogglePipelineCommand extends CommandBase {
  /** Creates a new TogglePipelineCommand. */
  private Vision m_Vision;

  public TogglePipelineCommand(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Vision = vision;
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Vision.togglePipeline();
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
