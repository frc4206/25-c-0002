// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Climber_Sub extends SubsystemBase {
  /** Creates a new climberSub. */
  DefaultTalonFX.Config climberMotorConfig1 = new DefaultTalonFX.Config("climber1Cfg");
  DefaultTalonFX.Config climberMotorConfig2 = new DefaultTalonFX.Config("climber2Cfg");
  CANcoder climberCCoder = new CANcoder(3);
  DigitalInput climerHalSensor = new DigitalInput(3);

  public class  Config  extends LoadableConfig {
    public double kHomePosition;

    public double kMaxReverseOutput;

    public double climberOutPos;
    public double climberInPos;
  //The next one probably won't be needed
    //public double climberDefaultPos;

    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX climberMotor1 = new DefaultTalonFX(climberMotorConfig1);
  public DefaultTalonFX climberMotor2 = new DefaultTalonFX(climberMotorConfig2);

  public Climber_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    climberMotor1.Duty_Cycle_Output(percentage);
    climberMotor2.Duty_Cycle_Output(percentage);
  }

  public void climberOutPos_func(double pos) {
    climberMotor1.PID_Position(pos);
    climberMotor2.PID_Position(pos);
  }

  public void climberInPos_func(double pos) {
    climberMotor1.PID_Position(pos);
    climberMotor2.PID_Position(pos);
  }
}
