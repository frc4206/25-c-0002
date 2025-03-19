// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PID_Commands.Arm_PID_React_Com;
import frc.robot.commands.PID_Commands.Elevator_PID_React_Com;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Elevator_Sub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4_scoring_React_Com extends ParallelCommandGroup {
  /** Creates a new L4_scoring_React_Com. */
  Elevator_Sub m_elevatorSub;
  Arm_Sub m_armSub;

  public L4_scoring_React_Com(Arm_Sub armSub, Elevator_Sub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_armSub = armSub;
    m_elevatorSub = elevatorSub;
    addCommands(new Arm_PID_React_Com(m_armSub, m_armSub.armConfig.l4ScoringPosition), new Elevator_PID_React_Com(elevatorSub, elevatorSub.elevatorConfig.l4ScoringPosition));
  }
}
