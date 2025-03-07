// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import org.team4206.battleaid.common.LoadableConfig;
import org.team4206.battleaid.common.TunedJoystick;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Swerve_PID extends Command {
  /** Creates a new Swerve_PID. */
  public static class Config extends LoadableConfig {

    double kpy;
    double kiy;
    double kdy;
    double ff;

    public Config(String filename) {

      super.load(this, filename);
      // LoadableConfig.print(this);
    }
  }

  double m_setpointY;

  static Config cfg = new Config("swervePID.toml");
  CommandSwerveDrivetrain m_drive;

  double MaxSpeed;
  double MaxAngularRate;


  double errorY;
  double lastErrorY = 0;
  double deltaY;

  TunedJoystick tj;

  public Swerve_PID(CommandSwerveDrivetrain drive, double setpointY, double sped, double angrate, TunedJoystick _tj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_setpointY = setpointY;
    tj = _tj;

    MaxSpeed = sped;
    MaxAngularRate = angrate;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    double feed_forward; 
    Pose3d pose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight-intake");
    errorY = pose.getX() - m_setpointY;
    double outputY = errorY * cfg.kpy;

    if (Math.abs(errorY) < 0.0181) {
      outputY = 0;
    }

    SmartDashboard.putNumber("output Y", outputY);
    SmartDashboard.putNumber("error y", errorY);
    if (outputY < 0) {
      cfg.ff = -cfg.ff;
    }
    // outputY = 0;



    SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric()
        //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(0.2)
        .withVelocityY(outputY + cfg.ff); // Use open-loop control for drive motors

      m_drive.setControl(driverequest);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
