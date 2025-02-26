// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Percent_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePercent_Com extends Command {
  /** Creates a new IntakeMoveCom. */
  Intake_Sub m_intakeSub;
  double m_percent;
  public IntakePercent_Com(Intake_Sub intakeSub, double percent) {
    m_intakeSub = intakeSub;
    m_percent = percent;
    addRequirements(m_intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("command intialized percent");
    m_intakeSub.setPercentage_func(m_percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setPercentage_func(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
