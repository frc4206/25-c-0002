// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1_scoring_Com extends Command {
  double m_intakePosition;
  Intake_Sub m_intakeSub;
  /** Creates a new L1_scoring_Com. */
  public L1_scoring_Com(Intake_Sub intakeSub, double intakePosition) {
    m_intakePosition = intakePosition;
    m_intakeSub = intakeSub;
    addRequirements(m_intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.setIntakePos_func(m_intakePosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
