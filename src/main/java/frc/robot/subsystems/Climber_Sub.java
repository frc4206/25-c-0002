// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Climber_Sub extends SubsystemBase {
  /** Creates a new climberSub. */
  DefaultTalonFX.Config climberMotorConfig1 = new DefaultTalonFX.Config("Climber1Motor.toml");
  DefaultTalonFX.Config climberMotorConfig2 = new DefaultTalonFX.Config("Climber2Motor.toml");
  public DefaultTalonFX climberMotor1 = new DefaultTalonFX(climberMotorConfig1);
  public DefaultTalonFX climberMotor2 = new DefaultTalonFX(climberMotorConfig2);
  DigitalInput climberHallSensor = new DigitalInput(3);
  public Config climberConfig = new Config("Climber.toml");
  public Config climberMotorConfig = new Config("Climber1Motor.toml");

  public TalonFX[] m_climberList = {climberMotor1.motor, climberMotor2.motor};
  public ArrayList<Double[]> currentLimitList = new ArrayList<>();
  

  public static class Config  extends LoadableConfig {
    public double stowPosition; 
    public double climbedFinalPosition; 
    public double climbReadyPosition; 
    public String name;

    public int canID; 

    public Config(String filename){

      super.load(this, filename);
        LoadableConfig.print(this);
    }
  }

  public Climber_Sub(Config cfg, Climber_Sub.Config climber_Motor_Config) {
    Double[] intakeLimits = {climberMotorConfig1.intakelimit, climberMotorConfig2.intakelimit};
    Double[] shootLimits = {climberMotorConfig1.shootlimit, climberMotorConfig2.shootlimit};
    Double[] climbLimits = {climberMotorConfig1.climblimit, climberMotorConfig2.climblimit};
    Double[] defenseLimits = {climberMotorConfig1.defenselimit, climberMotorConfig2.defenselimit};
    Double[] cycleLimits = {climberMotorConfig1.cyclelimit, climberMotorConfig2.cyclelimit};
    currentLimitList.add(intakeLimits);
    currentLimitList.add(shootLimits);
    currentLimitList.add(climbLimits);
    currentLimitList.add(defenseLimits);
    currentLimitList.add(cycleLimits);

    climberMotorConfig = climber_Motor_Config; 
    climberMotor2.motor.setControl(new Follower(climber_Motor_Config.canID, false));
  }

  public Climber_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub

    climberMotor1.Update_Sim();
    climberMotor2.Update_Sim();
    super.simulationPeriodic();
  }

  public void setPercentage_func(double percentage) {
    climberMotor1.Duty_Cycle_Output(percentage);
    climberMotor2.Duty_Cycle_Output(percentage);
  }

  public void setClimberPos_func(double pos) {
    climberMotor1.PID_Position(pos);
    climberMotor2.PID_Position(pos);
  }
}
