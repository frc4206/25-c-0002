// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Climber_Sub;
import frc.robot.subsystems.Elevator_Sub;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Deep_Climb_Com extends Command {
  Elevator_Sub m_elevatorSub;
  Arm_Sub m_armSub;
  Climber_Sub m_climberSub;
  Intake_Sub m_intakeSub;
  /** Creates a new Deep_Climb. */
  public Deep_Climb_Com(Intake_Sub intakeSub, Climber_Sub climberSub, Elevator_Sub elevatorSub, Arm_Sub armSub) {
    m_intakeSub = intakeSub;
    m_climberSub = climberSub;
    m_elevatorSub = elevatorSub; 
    m_armSub = armSub; 
    addRequirements(m_intakeSub);
    addRequirements(m_climberSub);
    addRequirements(m_elevatorSub);
    addRequirements(m_armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSub.setClimberPos_func(m_climberSub.climberConfig.climbReadyPosition);
    m_intakeSub.setIntakePos_func(m_intakeSub.intakeConfig.intakePosition);
    m_armSub.setArmAngle_func(m_armSub.armConfig.stowPosition);
    m_elevatorSub.setElevatorPos_func(m_elevatorSub.elevatorConfig.stowPosition);
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
