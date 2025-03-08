// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Claw_Sub.ClawState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClawStateCommand extends Command {
  /** Creates a new SetClawStateCommand. */
  Claw_Sub m_Claw_Sub;
  ClawState m_state;
  public SetClawStateCommand(Claw_Sub claw_Sub, ClawState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Claw_Sub = claw_Sub;
    m_state = state;
    addRequirements(claw_Sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Claw_Sub.setClawState(m_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
