// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PID_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Arm_PID_Com extends Command {
  double m_armPosition;
  Arm_Sub m_armSub;
  /** Creates a new Arm_PID_Com. */
  public Arm_PID_Com(Arm_Sub armSub, double position) {
    m_armPosition = position;
    m_armSub = armSub;
    addRequirements(m_armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSub.setArmAngle_func(m_armPosition);
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
