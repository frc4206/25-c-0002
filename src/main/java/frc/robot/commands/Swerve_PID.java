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

  private final double limelight_robot_offset = 0.05;

  /** Creates a new Swerve_PID. */
  public static class Config extends LoadableConfig {

    double kpy;
    double kiy;
    double kdy;
    double ff;

    double kddiff;

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
    Pose3d pose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight-intake");

    double sag_output = 0.0d;
    double x_output = 0.0d;

    // from the limelights perspective, Y is the nearness,
    // zero is on top of, farther away goes negative
    double distance_to_qr_code = pose.getZ();

    // left is positive, right is negative
    // from the limelights perspective, X is lefty-rightness
    double central_alignment = pose.getX() - limelight_robot_offset; 

    // the closest we can bot on robot perimeter is ~-0.56
    // so we are gonna round down to -0.5
    if(distance_to_qr_code >= -0.5d){
      // may need to break here
      sag_output = 0.0d;
    } else {
      sag_output = -distance_to_qr_code;
    }

    // IF we are detecting the april tag
    if(LimelightHelpers.getTV("limelight-intake")){
      // alignment is still a function of the setpoint
      central_alignment -= m_setpointY;

      // Adding P
      x_output += (central_alignment * cfg.kpy);


      double diff = central_alignment - lastErrorY;

      x_output += (diff * cfg.kddiff);

      // if they are not the same, it means 
      // that we need to apply a derivative error, 'diff'
      // diff = central_alignment - lastErrorY;

      // this OPPOSES the proportional value
      // x_output += (diff * cfg.kddiff);

      SmartDashboard.putNumber("Xoutput: ", x_output);
      SmartDashboard.putNumber("Diff (d): ", diff);
      SmartDashboard.putNumber("Central alignment 1:", central_alignment);
      SmartDashboard.putNumber("Central alignment 2:", lastErrorY);
    }

    SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityY(x_output);
        // .withVelocityX(sag_output); // Use open-loop control for drive motors
        if (!LimelightHelpers.getTV("limelight-intake") && m_setpointY < 0) {
          driverequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityY(-0.5);
        }
        if (!LimelightHelpers.getTV("limelight-intake") && m_setpointY > 0) {
          driverequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityY(0.5);
        }
    m_drive.setControl(driverequest);

    lastErrorY = central_alignment;
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
