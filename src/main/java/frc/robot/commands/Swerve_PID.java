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
    double kpx;
    double kix;
    double kdx;

    double kpy;
    double kiy;
    double kdy;

    double kptheta;
    double kitheta;
    double kdtheta;

    public Config(String filename) {

      super.load(this, filename);
      // LoadableConfig.print(this);
    }
  }

  double m_setpointX;
  double m_setpointY;
  double m_setpointTheta;

  static Config cfg = new Config("swervePID.toml");
  CommandSwerveDrivetrain m_drive;

  double MaxSpeed;
  double MaxAngularRate;

  double errorX;
  double lastErrorX = 0;
  double deltaX;

  double errorY;
  double lastErrorY = 0;
  double deltaY;

  double errorTheta;
  double lastErrorTheta = 0;
  double deltaTheta;

  TunedJoystick tj;

  public Swerve_PID(CommandSwerveDrivetrain drive, double setpointX, double setpointY, double sped, double angrate, TunedJoystick _tj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_setpointX = setpointX;
    m_setpointY = setpointY;
    tj = _tj;

    MaxSpeed = sped;
    MaxAngularRate = angrate;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_setpointTheta = getTagThetaSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.

  public double getTagThetaSetpoint() {
    int tagID = (int) LimelightHelpers.getFiducialID("limelight-intake");
    if (tagID == 18) {
      return 0;
    } else {
      return 0;
    }
  }
  @Override
  public void execute() {
    Pose3d pose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight-intake");
    double theta = m_drive.getPigeon2().getRotation2d().getDegrees();
    errorX = pose.getZ() - m_setpointX;
    deltaX = errorX - lastErrorX;

    errorY = pose.getX() - m_setpointY;
    deltaY = errorY - lastErrorY;

    errorTheta = theta - m_setpointTheta;
    deltaTheta = errorTheta - lastErrorTheta;

    double outputX = -errorX * cfg.kpx - deltaX * cfg.kdx;
    double outputY = errorY * cfg.kpy + deltaY * cfg.kdy;

    if (Math.abs(errorX) < 0.0181) {
      outputX = 0;
    }

    if (Math.abs(errorY) < 0.0181) {
      outputY = 0;
    }

    // SmartDashboard.putNumber("pid x", -errorX * cfg.kpx);
    // SmartDashboard.putNumber("error x", errorX);

    // SmartDashboard.putNumber("pid D", -deltaX * cfg.kdx);
    // SmartDashboard.putNumber("delta x", deltaX);

    SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(outputX)
        .withVelocityY(outputY)
        .withRotationalRate(-errorTheta * cfg.kptheta); // Use open-loop control for drive motors
    SwerveRequest.RobotCentric driverequesttheta = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(-errorTheta * cfg.kptheta); // Use open-loop control for drive motors

    // m_drive.applyRequest(() -> driverequest.withVelocityX(errorX * kpx)
    // .withVelocityY(-tj.getLeftX() * MaxSpeed)
    // .withRotationalRate(-tj.getRightX() * MaxAngularRate));
    if (Math.abs(errorTheta) < 5) {
      m_drive.setControl(driverequest);
    }else {
      m_drive.setControl(driverequesttheta);
    }
    
    lastErrorX = errorX;
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
