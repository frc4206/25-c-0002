// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.DefaultTalonFX;

public class Climber_Sub extends SubsystemBase {
  /** Creates a new climberSub. */

  /*Configs */
  ConfigTalonFX.Config climberMotorConfig1 = new ConfigTalonFX.Config("Climber1Motor.toml");
  ConfigTalonFX.Config climberMotorConfig2 = new ConfigTalonFX.Config("Climber2Motor.toml");
  public Config climberConfig;

  /*Motors */
  public TalonFX climberMotor1 = new TalonFX(climberMotorConfig1.canID);
  public TalonFX climberMotor2 = new TalonFX(climberMotorConfig2.canID);

  /*Sensors */
  DigitalInput climberHallSensor = new DigitalInput(3);

  /*Game state Lists */
  public TalonFX[] m_climberList = {climberMotor1, climberMotor2};
  public ArrayList<Double[]> currentLimitList = new ArrayList<>();
  

  public static class Config  extends LoadableConfig {
    /*IDs and Ports */
    public String name;

    /*Positions */
    public double stowPosition; 
    public double climbedFinalPosition; 
    public double climbReadyPosition; 
    
    /*Misc. */

    public Config(String filename){

      super.load(this, filename);
        LoadableConfig.print(this);
    }
  }

  public Climber_Sub(Config cfg) {
    /*Game State Constants */
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

    climberMotor2.setControl(new Follower(climberMotorConfig1.canID, false));
  }
  

  public void setPercentage_func(double percentage) {
    climberMotor1.setControl(new DutyCycleOut(percentage));
  }

  public void setClimberPos_func(double pos) {
    climberMotor1.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
