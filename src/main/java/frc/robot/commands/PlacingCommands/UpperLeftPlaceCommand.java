// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacingCommands;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ArmAndClawSub;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import utilities.ConfigurablePID;
import utilities.AutoAlignment;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpperLeftPlaceCommand extends CommandBase {
  /** Creates a new UpperLeftPlaceCommand. */

  private DriveTrainSub m_driveTrainSub;
  private ArmAndClawSub m_armAndClawSub;
  private Vision m_vision;

  private boolean atPosition = false;
  private boolean shouldEnd;

  private AutoAlignment autoAlignment;

  public UpperLeftPlaceCommand(DriveTrainSub driveTrainSub, ArmAndClawSub armAndClawSub, Vision vision) {
    m_driveTrainSub = driveTrainSub;
    m_armAndClawSub = armAndClawSub;
    m_vision = vision;

    autoAlignment = new AutoAlignment(m_driveTrainSub, m_vision, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSub, m_armAndClawSub, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shouldEnd = !m_vision.isTrackedAprilTagFound();
    autoAlignment.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 20.5 deg vertical field of view.

    atPosition = autoAlignment.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
