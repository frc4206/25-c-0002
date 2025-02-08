// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Elevator_Sub;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_Intake_Com extends Command {

  Elevator_Sub m_elevatorSub;
  Intake_Sub m_intakeSub;
  Arm_Sub m_armSub;
  double m_elevatorPosition;
  double m_armPosition;
  double m_intakePosition;
  /** Creates a new Coral_Intake. */
  public Coral_Intake_Com(Arm_Sub armSub, Intake_Sub intakeSub, Elevator_Sub elevatorSub, double elevatorPosition, double armPosition, double intakePosition) {
    m_elevatorPosition = elevatorPosition;
    m_armPosition = armPosition;
    m_intakePosition = intakePosition;
    m_armSub = armSub;
    m_intakeSub = intakeSub;
    m_elevatorSub = elevatorSub;
    addRequirements(m_armSub);
    addRequirements(m_intakeSub);
    addRequirements(m_elevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSub.setArmAngle_func(m_armPosition);
    m_elevatorSub.setElevatorPos_func(m_elevatorPosition);
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
