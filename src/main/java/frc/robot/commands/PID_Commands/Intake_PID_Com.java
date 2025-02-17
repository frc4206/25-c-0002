// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PID_Com;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake_PID_Com extends Command {
  Intake_Sub m_intakeSub;
  double m_intakePosition;
  /** Creates a new Intake_PID_Com. */
  public Intake_PID_Com(Intake_Sub intakeSub, double intakePosition) {
    m_intakeSub = intakeSub;
    m_intakePosition = intakePosition;
    // Use addRequirements() here to declare subsystem dependencies.
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
  public void end(boolean interrupted) {
    m_intakeSub.setIntakePos_func(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
