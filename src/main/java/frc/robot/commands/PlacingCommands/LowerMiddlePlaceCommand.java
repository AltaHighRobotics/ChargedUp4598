// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ArmAndClawSub;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import utilities.ConfigurablePID;
import utilities.AutoAlignment;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class LowerMiddlePlaceCommand extends CommandBase {
  /** Creates a new UpperLeftPlaceCommand. */

  private XboxController m_driveController;
  private DriveTrainSub m_driveTrainSub;
  private ArmAndClawSub m_armAndClawSub;
  private Vision m_vision;

  private boolean atPosition = false;
  private boolean shouldEnd;

  private AutoAlignment autoAlignment;

  private long startTime;

  private int stage;

  public LowerMiddlePlaceCommand(XboxController driveController, DriveTrainSub driveTrainSub, ArmAndClawSub armAndClawSub, Vision vision) {
    m_driveTrainSub = driveTrainSub;
    m_armAndClawSub = armAndClawSub;
    m_vision = vision;
    m_driveController = driveController;

    autoAlignment = new AutoAlignment(m_driveTrainSub, m_vision, Constants.MIDDLE_VERTICAL_SETPOINT, Constants.MIDDLE_HORIZONTAL_SETPOINT, true);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSub, m_armAndClawSub, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setLimelightPipeline(Constants.LIMELIGHT_APRIL_TAG_PIPELINE);
    shouldEnd = !m_vision.isTargetFound();
    autoAlignment.reset();
    stage = 0;

    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 20.5 deg vertical field of view.
    // FIX MAGIC NUMBERS
    
    // If driver stops this command.
    if (m_driveController.getRawButtonPressed(Constants.STOP_PLACE_COMMAND_BUTTON)) {
      shouldEnd = true;
      return;
    }
    
    switch (stage) {
      case 0: // Align robot with target.
        atPosition = autoAlignment.run();

        if (atPosition) {
          stage = 1;
        }

        break;
      case 1: // Raise arm to position.
        m_armAndClawSub.armLower();

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 2;
        }

        break;
      case 2: // Open claw and wait for it.
        m_armAndClawSub.clawOpen();

        // Get start time.
        if (startTime == 0) {
          startTime = System.currentTimeMillis();
        }

        // Next stage.
        if (System.currentTimeMillis() - startTime >= Constants.WAIT_TIME) {
          stage = 3;
        }

        break;
      default:
        shouldEnd = true;
        break;
    }

    SmartDashboard.putNumber("Stage", stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSub.stop();
    m_armAndClawSub.armRest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
