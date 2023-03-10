// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveTrainSub;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroAngleCommand extends CommandBase {
  /** Creates a new ZeroAngleCommand. */
  private DriveTrainSub m_driveTrainSub;

  public ZeroAngleCommand(DriveTrainSub driveTrainSub) {
    m_driveTrainSub = driveTrainSub;
    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrainSub.resetGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
