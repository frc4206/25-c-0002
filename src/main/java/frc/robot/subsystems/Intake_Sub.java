// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Intake_Sub extends SubsystemBase {
  /** Creates a new intakeSub. */
  DefaultTalonFX.Config intakeMotorRollersConfig = new DefaultTalonFX.Config("intakeRollersCfg");
  DefaultTalonFX.Config intakeMotorPivotConfig = new DefaultTalonFX.Config("intakePivotCfg");
  DigitalInput intakeHallSensor = new DigitalInput(6);
  DigitalInput intakeBeamBreak = new DigitalInput(7);
  public Config intakeConfig = new Config("Intake.toml");


  public class  Config  extends LoadableConfig {
    public double stowPosition; 
    public double l1ScoringPosition; 
    public double intakePosition;
    public double intakePercent; 
    public double outtakePercent; 

    public Config(String filename){
      
      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX intakeMotorRollers = new DefaultTalonFX(intakeMotorRollersConfig);
  public DefaultTalonFX intakeMotorPivot = new DefaultTalonFX(intakeMotorPivotConfig);

  public Intake_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    intakeMotorRollers.Duty_Cycle_Output(percentage);
  }

  public void setIntakePos_func(double pos) {
    intakeMotorPivot.PID_Position(pos);
  }
}
