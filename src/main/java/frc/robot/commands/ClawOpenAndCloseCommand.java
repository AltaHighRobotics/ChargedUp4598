// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmAndClawSub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class ClawOpenAndCloseCommand extends CommandBase {
  /** Creates a new ClawOpenAndCloseCommand. */
  private ArmAndClawSub m_armAndClawSub;
  private Compressor phCompressor;

  public ClawOpenAndCloseCommand() {
    phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armAndClawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_armAndClawSub.clawOpen();
    phCompressor.enableDigital();

    boolean enabled = phCompressor.enabled();
    boolean pressureSwitch = phCompressor.getPressureSwitchValue();
    double current = phCompressor.getCurrent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armAndClawSub.clawClose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
