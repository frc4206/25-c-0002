// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Percent_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Claw_Sub.ClawState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawPercent_Com extends Command {
  /** Creates a new ClawCom. */
  Claw_Sub m_clawSub;
  double m_percent;

  public ClawPercent_Com(Claw_Sub clawSub, double percent) {
    m_clawSub = clawSub;
    m_percent = percent;
    addRequirements(m_clawSub);
  }

  @Override
  public void initialize(){
    ClawState current = Claw_Sub.getClawState();
    if(current == ClawState.NEUTRAL){
      Claw_Sub.setClawState(ClawState.INTAKING);
      m_clawSub.setPercentage_func(m_percent);
    } else if(current == ClawState.DETECTED){
      Claw_Sub.setClawState(ClawState.EXHAUSTING);
      m_clawSub.setPercentage_func(m_percent);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClawState current = Claw_Sub.getClawState();
    if (current == ClawState.INTAKING) {
      m_clawSub.setPercentage_func(m_percent);
    } else if(current == ClawState.DETECTED){
      m_clawSub.setPercentage_func(0);
    }
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
