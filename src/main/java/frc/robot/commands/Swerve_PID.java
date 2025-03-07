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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Swerve_PID extends Command {

  /*
   * The (bottom) Limelight lens was measured at 5 centimeters
   * starboard from the bow-stern axis.
   */
  private final double limelight_starboard_offset = 0.05; // in centimeters
  private double last_alignment_measurement = 0.0;

  /**
   * The (bottom) Limelight lens was measure at 57.15 centimeters
   * from the bow of the robot (including the bumper on the robot).
   * From the Limelight's perspective, forward direction is negative
   */
  private final double limelight_bow_offset = -0.5715; // in centimeters
  private double last_distance_measurement = 0.0;

  /** Creates a new Swerve_PID. */
  public static class Config extends LoadableConfig {

    public double kp_strafe;
    public double kd_strafe;

    public double kp_distance;
    public double kd_distance;

    public Config(String filename) {
      super.load(this, filename);
    }
  }

  static Config cfg = new Config("swervePID.toml");
  CommandSwerveDrivetrain m_drive;

  // offset left or right comes from RobotContainer
  private double m_setpointY;

  public Swerve_PID(CommandSwerveDrivetrain drive, double setpointY) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_setpointY = setpointY;

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

    double forward_speed = 0.0d;
    double x_output = 0.0d;

    // from the limelights perspective, Y is the nearness,
    // zero is on top of, farther away goes negative
    double distance_to_qr_code = pose.getZ() - limelight_bow_offset;

    // left is positive, right is negative
    // from the limelights perspective, X is lefty-rightness
    double central_alignment = pose.getX() - limelight_starboard_offset; 

    // IF we are detecting the april tag
    if(LimelightHelpers.getTV("limelight-intake")){

      /*---------- move sideways ------------------- */
      
      // Left-rightness is function of the setpoint
      central_alignment -= m_setpointY;

      // Adding P
      x_output += (central_alignment * cfg.kp_strafe);

      // Rough estimate calculation of D
      double stafe_d = central_alignment - last_alignment_measurement;

      // Adding D
      x_output += (stafe_d * cfg.kd_strafe);

      /* ------------------------------------------- */

      /* ----------------move forwards---------------- */

      // Adding P
      forward_speed += (distance_to_qr_code * cfg.kp_distance);

      // Rought estimate calculation of D
      double forward_d = distance_to_qr_code - last_distance_measurement;

      // Adding D
      forward_speed += (forward_d * cfg.kd_distance);
    }

    SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityY(x_output)
        .withVelocityX(forward_speed);

    m_drive.setControl(driverequest);

    // Make sure we remember last alignment value
    // so we can roughly calculate D
    last_alignment_measurement = central_alignment;
    last_distance_measurement = distance_to_qr_code;
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
