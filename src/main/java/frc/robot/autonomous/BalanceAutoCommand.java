// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSub;
import utilities.ConfigurablePID;
import frc.robot.Constants;

public class BalanceAutoCommand extends CommandBase {
  /** Creates a new BalanceAutoCommand. */
  private DriveTrainSub m_driveTrainSub;

  private ConfigurablePID distancePid;
  private ConfigurablePID balancePid;

  private int stage;
  private boolean finished;

  private double speed;

  public BalanceAutoCommand(DriveTrainSub driveTrainSub) {
    m_driveTrainSub = driveTrainSub;

    distancePid = new ConfigurablePID(Constants.AUTONOMOUS_DISTANCE_PID);
    balancePid = new ConfigurablePID(Constants.AUTONOMOUS_BALANCE_PID);

    addRequirements(m_driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    finished = false;

    distancePid.resetValues();
    balancePid.resetValues();

    m_driveTrainSub.resetWheelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (stage) {
      case 0: // Drive to platform.
        speed = distancePid.runPID(Constants.BALANCE_AUTO_DISTANCE, m_driveTrainSub.getAvgWheelEncoder());
        m_driveTrainSub.setSwerveDrive(0.0, speed, 0.0, false);

        // Next stage
        if (distancePid.getError() == 0.0) {
          stage = 1;
        }

        break;
      case 1: // Balance on platform.
        speed = balancePid.runPID(0.0, m_driveTrainSub.getPitch());
        m_driveTrainSub.setSwerveDrive(0.0, speed, 0.0, false);

        break;
      default:
        finished = true;
        break;
    }

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
