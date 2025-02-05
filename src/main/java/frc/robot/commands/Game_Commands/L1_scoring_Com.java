// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1_scoring_Com extends Command {
  double m_clawPosition;
  double m_intakePosition;
  Claw_Sub m_clawSub;
  Intake_Sub m_intakeSub;
  /** Creates a new L1_scoring_Com. */
  public L1_scoring_Com(Intake_Sub intakeSub, Claw_Sub clawSub, double clawPosition, double intakePosition) {
    m_clawPosition = clawPosition;
    m_intakePosition = intakePosition;
    m_intakeSub = intakeSub;
    m_clawSub = clawSub;
    addRequirements(m_intakeSub);
    addRequirements(m_clawSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.intakeOutPos_func(m_intakePosition);
    m_clawSub.reefTroughAngle_func(m_clawPosition);
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
