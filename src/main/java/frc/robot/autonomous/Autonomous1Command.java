// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSub;
import frc.robot.subsystems.ArmAndClawSub;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import utilities.ConfigurablePID;
import utilities.AutoAlignment;
import frc.robot.Constants;

public class Autonomous1Command extends CommandBase {
  /** Creates a new Autonomous1Command. */
  private DriveTrainSub m_driveTrainSub;
  private ArmAndClawSub m_armAndClawSub;
  private Vision m_vision;

  private boolean atPosition = false;
  private boolean shouldEnd;

  private AutoAlignment autoAlignment;
  private AutoAlignment driveBackAutoAlignment;

  private long startTime;

  private int stage;

  public Autonomous1Command(DriveTrainSub driveTrainSub, ArmAndClawSub armAndClawSub, Vision vision) {
    m_driveTrainSub = driveTrainSub;
    m_armAndClawSub = armAndClawSub;
    m_vision = vision;

    autoAlignment = new AutoAlignment(m_driveTrainSub, m_vision, Constants.MIDDLE_VERTICAL_SETPOINT, Constants.MIDDLE_HORIZONTAL_SETPOINT, true);
    driveBackAutoAlignment = new AutoAlignment(m_driveTrainSub, m_vision, Constants.DRIVE_BACK_VERTICAL_SETPOINT, Constants.DRIVE_BACK_HORIZONTAL_SETPOINT, true);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSub, m_armAndClawSub, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setLimelightPipeline(Constants.LIMELIGHT_APRIL_TAG_PIPELINE);
    //shouldEnd = !m_vision.isTargetFound();
    shouldEnd = false;

    autoAlignment.reset();
    driveBackAutoAlignment.reset();

    stage = 0;

    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 20.5 deg vertical field of view.
    // FIX MAGIC NUMBERS
    switch (stage) {
      case 0: // Go to arm rest position.
        m_armAndClawSub.armRest();

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 1;
        }
        
        break;
      case 1: // Open claw and go to grab position.
        m_armAndClawSub.clawOpen();
        m_armAndClawSub.armGrab();

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 2;
        }
        
        break;
      case 2: // Close claw and wait.
        m_armAndClawSub.clawClose();

        // Get start time.
        if (startTime == 0) {
          startTime = System.currentTimeMillis();
        }

        // Next stage.
        if (System.currentTimeMillis() - startTime >= Constants.WAIT_TIME) {
          stage = 3;
          startTime = 0; // Reset start time.
        }

        break;
      case 3: // Align robot with target and arm rest.
        m_armAndClawSub.armRest();
        atPosition = autoAlignment.run();

        if (atPosition) {
          stage = 4;
        }

        break;
      case 4: // Raise arm to position.
        m_armAndClawSub.armHigher();

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 5;
        }

        break;
      case 5: // Open claw and wait for it.
        m_armAndClawSub.clawOpen();

        // Get start time.
        if (startTime == 0) {
          startTime = System.currentTimeMillis();
        }

        // Next stage.
        if (System.currentTimeMillis() - startTime >= Constants.WAIT_TIME) {
          stage = 6;
        }

        break;
      case 6: // Drive back and arm rest.
        m_armAndClawSub.armRest();
        atPosition = driveBackAutoAlignment.run();

        if (atPosition && m_armAndClawSub.isAtFinalPosition()) {
          stage = 7;
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
    m_armAndClawSub.clawOpen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
