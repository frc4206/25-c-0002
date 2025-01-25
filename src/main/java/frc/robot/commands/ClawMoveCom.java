// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawMoveCom extends Command {
  /** Creates a new ClawCom. */
  ClawSub m_clawSub;
  double m_percent;
  public ClawMoveCom(ClawSub clawSub, double percent) {
    m_clawSub = clawSub;
    m_percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clawSub.setPercentage_func(m_percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSub.setPercentage_func(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
