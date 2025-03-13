// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PID_Commands.Arm_PID_Com;
import frc.robot.commands.PID_Commands.Elevator_PID_Com;
import frc.robot.commands.Percent_Commands.ClawPercent_Com;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Elevator_Sub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Coral_Intake_Com extends ParallelCommandGroup {
  /** Creates a new Coral_Intake_Com. */
  public Coral_Intake_Com(Arm_Sub armSub, Claw_Sub clawSub, Elevator_Sub elevatorSub) {
    addCommands(new Arm_PID_Com(armSub, armSub.armConfig.sourceIntakePosition),
        new Elevator_PID_Com(elevatorSub, elevatorSub.elevatorConfig.sourceIntakePosition),
        new ClawPercent_Com(clawSub, clawSub.clawConfig.intakePercent));
  }
}
