// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Game_Commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwervePPAlign extends Command {
  /** Creates a new SwervePPAlign. */
  CommandSwerveDrivetrain m_drivetrain;
  String pathname;
  double tagid;
  boolean isfin = false;

  public SwervePPAlign(CommandSwerveDrivetrain drivetrain, String pathname) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    this.pathname = pathname;
    // addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command run");
    String pathstr = "6" + pathname;
    tagid = m_drivetrain.tagid;
    // try {
    // System.out.println("try catch run");
    // if (tagid == 18) {
    // pathstr = "6" + pathname;
    // System.out.println("id 18, path: " + pathstr);

    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathstr);

    // // Create a path following command using AutoBuilder. This will also trigger
    // // event markers.
    // AutoBuilder.followPath(path);
    // } else if (tagid == 19) {
    // pathstr = "8" + pathname;
    // System.out.println("id 19, path: " + pathstr);
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathstr);

    // // Create a path following command using AutoBuilder. This will also trigger
    // // event markers.
    // AutoBuilder.followPath(path);
    // }
    // } catch (IOException e) {

    // } catch (ParseException e) {
    // // TODO Auto-generated catch block
    // e.printStackTrace();
    // }

    if (tagid == 18 || tagid == 7) {
      pathstr = "6" + pathname;
      System.out.println("id 18, path: " + pathstr);

    } else if (tagid == 19 || tagid == 6) {
      pathstr = "8" + pathname;
      System.out.println("id 19, path: " + pathstr);
    }else if (tagid == 20 || tagid == 11) {
      pathstr = "10" + pathname;
      System.out.println("id 20, path: " + pathstr);

    }else if (tagid == 21 || tagid == 10) {
      pathstr = "12" + pathname;
      System.out.println("id 21, path: " + pathstr);

    }else if (tagid == 22 || tagid == 9) {
      pathstr = "2" + pathname;
      System.out.println("id 22, path: " + pathstr);

    }else if (tagid == 17 || tagid == 8) {
      pathstr = "4" + pathname;
      System.out.println("id 17, path: " + pathstr);
    }

    System.out.println(m_drivetrain.pathstr);

    m_drivetrain.pathstr = pathstr;

    m_drivetrain.followPathCommand(pathstr).schedule();
    // isfin = true;
    // isFinished();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfin;
  }
}
