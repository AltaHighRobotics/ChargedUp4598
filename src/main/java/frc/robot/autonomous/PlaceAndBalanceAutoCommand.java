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
import frc.robot.Constants;

public class PlaceAndBalanceAutoCommand extends CommandBase {
  /** Creates a new PlaceAndBalanceAutoCommand. */
  private DriveTrainSub m_driveTrainSub;
  private ArmAndClawSub m_armAndClawSub;

  private boolean atPosition = false;
  private boolean shouldEnd;

  private ConfigurablePID balancePid;

  private long startTime;

  private double speed;

  private int stage;

  public PlaceAndBalanceAutoCommand(DriveTrainSub driveTrainSub, ArmAndClawSub armAndClawSub) {
    m_driveTrainSub = driveTrainSub;
    m_armAndClawSub = armAndClawSub;

    balancePid = new ConfigurablePID(Constants.AUTONOMOUS_BALANCE_PID);

    addRequirements(m_driveTrainSub, m_armAndClawSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    shouldEnd = false;
    balancePid.resetValues();
    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
      case 0: // claw close and arm rest
        m_armAndClawSub.armRest();
        m_armAndClawSub.setClawTwoMotor(Constants.CLAW_IDLE_SPEED);

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 1;
        }
      
        break;
      case 1: // Raise arm to position.
        m_armAndClawSub.armHigher();

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 2;
        }

        break;
      case 2: // Open claw and wait for it.
        m_armAndClawSub.setClawTwoMotor(Constants.CLAW_SPIT_SPEED);

        // Get start time.
        if (startTime == 0) {
          startTime = System.currentTimeMillis();
        }

        // Next stage.
        if (System.currentTimeMillis() - startTime >= Constants.WAIT_TIME) {
          stage = 3;
          startTime = 0;
        }

        break;
      case 3: // Back to rest
        m_armAndClawSub.armRest();
        m_armAndClawSub.setClawTwoMotor(Constants.CLAW_IDLE_SPEED);

        if (m_armAndClawSub.isAtFinalPosition()) {
          stage = 4;
        }

        break;
      case 4: // Drive to platform.
        speed = Constants.BALANCE_AUTO_SPEED;
        m_driveTrainSub.setSwerveDrive(0.0, speed, 0.0, false, false);

        // Next stage
        if (Math.abs(m_driveTrainSub.getPitch()) >= Constants.BALANCE_AUTO_DRIVE_TO_ANGLE) {
          stage = 5;
          m_driveTrainSub.stop();
        }

        break;
      case 5: // Balance on platform.
        speed = balancePid.runPID(0.0, m_driveTrainSub.getPitch());
        m_driveTrainSub.setSwerveDrive(0.0, speed, 0.0, false, false);
        break;
      default:
        shouldEnd = true;
        break;
    }

    m_driveTrainSub.run();
    SmartDashboard.putNumber("Stage", stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
