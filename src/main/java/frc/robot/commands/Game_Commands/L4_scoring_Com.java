// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Elevator_Sub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4_scoring_Com extends Command {
  //TODO: double check if claw inputs are needed in this command
  double m_elevatorPosition;
  double m_armPosition;
  Elevator_Sub m_elevatorSub;
  Arm_Sub m_armSub;
  /** Creates a new L4scoring. */
  public L4_scoring_Com(Arm_Sub armSub, Elevator_Sub elevatorSub, double armPosition, double elevatorPosition) {
    m_armPosition = armPosition;
    m_elevatorPosition = elevatorPosition;
    m_armSub = armSub;
    m_elevatorSub = elevatorSub;
    addRequirements(m_armSub);
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
