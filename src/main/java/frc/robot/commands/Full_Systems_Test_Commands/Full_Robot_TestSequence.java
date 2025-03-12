// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Full_Systems_Test_Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Game_Commands.Coral_Intake_Com;
import frc.robot.commands.Game_Commands.L1_scoring_Com;
import frc.robot.commands.Game_Commands.L2_scoring_Com;
import frc.robot.commands.Game_Commands.L3_scoring_Com;
import frc.robot.commands.Game_Commands.L4_scoring_Com;
import frc.robot.commands.PID_Commands.Climber_PID_Com;
import frc.robot.commands.PID_Commands.Intake_PID_Com;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Climber_Sub;
import frc.robot.subsystems.Elevator_Sub;
import frc.robot.subsystems.Intake_Sub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Full_Robot_TestSequence extends SequentialCommandGroup {
  /** Creates a new Full_Robot_TestSequence. */
  Intake_Sub m_intakeSub;
  Climber_Sub m_climberSub;
  Elevator_Sub m_elevatorSub;
  Claw_Sub m_clawSub;
  Arm_Sub m_armSub;

  public Full_Robot_TestSequence(Arm_Sub armSub, Claw_Sub clawSub, Elevator_Sub elevatorSub, Climber_Sub climberSub, Intake_Sub intakeSub, double intakeOutPos, double intakeInPos, double climberOutPos, double climberInPos, double climberFinPos) {
    m_intakeSub = intakeSub;
    m_climberSub = climberSub;
    m_armSub = armSub;
    m_clawSub = clawSub;
    m_elevatorSub = elevatorSub;
    addRequirements(m_intakeSub);
    addRequirements(m_climberSub);
    addRequirements(m_armSub);
    addRequirements(m_clawSub);
    addRequirements(m_elevatorSub);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Coral_Intake_Com(), 
        new L4_scoring_Com(m_armSub, m_clawSub, m_elevatorSub),
        new L3_scoring_Com(m_armSub, m_clawSub, m_elevatorSub),
        new L2_scoring_Com(m_armSub, m_clawSub, m_elevatorSub),
        new L1_scoring_Com(intakeSub),
        new Intake_PID_Com(intakeSub, intakeOutPos),
        new Intake_PID_Com(intakeSub, intakeInPos),
        new Intake_PID_Com(intakeSub, intakeOutPos),
        new Climber_PID_Com(climberSub, climberOutPos),
        new Climber_PID_Com(climberSub, climberInPos),
        new Climber_PID_Com(climberSub, climberFinPos)
        );

    
  //feed, the 4 Ls, intake in&out, intake out, climber in&out.
  }
}
