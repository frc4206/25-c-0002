// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Elevator_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4_scoring_Com extends Command {
  Elevator_Sub m_elevatorSub;
  Arm_Sub m_armSub;
  Claw_Sub m_clawSub;
  
  /** Creates a new L4scoring. */
  public L4_scoring_Com(Arm_Sub armSub, Claw_Sub clawSub, Elevator_Sub elevatorSub) {
    m_armSub = armSub;
    m_elevatorSub = elevatorSub;
    m_clawSub = clawSub;
    addRequirements(m_armSub);
    addRequirements(m_elevatorSub);
    addRequirements(m_clawSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSub.setArmAngle_func(m_armSub.armConfig.l4ScoringPosition);
    m_elevatorSub.setElevatorPos_func(m_elevatorSub.elevatorConfig.l4ScoringPosition );
    // m_clawSub.setPercentage_func(m_clawSub.clawConfig.outtakePercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
