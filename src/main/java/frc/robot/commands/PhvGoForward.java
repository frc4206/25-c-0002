// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team4206.battleaid.common.LoadableConfig;
import org.team4206.battleaid.common.TunedJoystick;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PhvGoForward extends Command {
  /** Creates a new PhvGoForward. */
  private final double camera_robot_offset = 0.05; 
  public boolean isFinished = false; 
  PhotonCamera camera = new PhotonCamera("robovikes4206"); 
  PhotonTrackedTarget target; 

  public static class Config extends LoadableConfig {

    double kpy;
    double kiy; 
    double kdy; 
    double ff; 

    double kddiff; 

    public Config(String filename) {
      super.load(this, filename); 
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

  public PhvGoForward(CommandSwerveDrivetrain drive, double setpointY, double sped, double angrate, TunedJoystick _tj) {
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sag_output = 0.0d; 
    double x_output = 0.0d; 

    var result = camera.getLatestResult(); 
    boolean hasTargets = result.hasTargets(); 

    PhotonTrackedTarget target = result.getBestTarget(); 

    if (target != null) {

      //X is forward
      double distance_to_qr_code = target.getBestCameraToTarget().getX(); 

      double central_alignment = target.getBestCameraToTarget().getY() - camera_robot_offset;

      //if detecting april tag 
      if (distance_to_qr_code >= -0.5d) {
        sag_output = 0.0d; 
      } else {
        sag_output = -distance_to_qr_code; 
      }

      if (hasTargets) {
        central_alignment -= m_setpointY; 

        x_output += (central_alignment * cfg.kpy); 

        double diff = central_alignment - lastErrorY; 

        x_output += (diff * cfg.kddiff); 
      }

      SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric().
      withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(distance_to_qr_code); 

      // lastErrorY = central_alignment; 
      if (Math.abs(distance_to_qr_code) > 0.025) {
        driverequest = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(0.5); 
      }

      if (Math.abs(distance_to_qr_code) < 0.025) {
        driverequest = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(0); 
        m_drive.setControl(driverequest); 
        isFinished = true; 
        isFinished();
      }

      m_drive.setControl(driverequest); 

    }
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
