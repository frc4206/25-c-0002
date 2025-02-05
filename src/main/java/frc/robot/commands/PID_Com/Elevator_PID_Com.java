// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PID_Com;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator_PID_Com extends Command {

  Elevator_Sub m_elevatorSub;
  double m_elevatorPosition;
  /** Creates a new Elevator_PID_Com. */
  public Elevator_PID_Com(Elevator_Sub elevatorSub, double position) {  ;
    m_elevatorSub = elevatorSub;
    m_elevatorPosition = position;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSub);
  }

    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSub.elevatorMoveToPos_func(m_elevatorPosition);
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
