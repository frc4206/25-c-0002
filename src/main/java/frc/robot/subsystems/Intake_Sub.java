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
  DefaultTalonFX.Config intakeMotorConfig1 = new DefaultTalonFX.Config("intake1Cfg");
  DefaultTalonFX.Config intakeMotorConfig2 = new DefaultTalonFX.Config("intake2Cfg");
  CANcoder intakeCCoder = new CANcoder(62);
  DigitalInput intakeHallSensor = new DigitalInput(6);
  DigitalInput intakeBeamBreak = new DigitalInput(7);

  public class  Config  extends LoadableConfig {
    public double kHomePosition;

    public double intakeUpPos;
    public double intakeOutPos;

    public Config(String filename){
      
      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX intakeMotor1 = new DefaultTalonFX(intakeMotorConfig1);
  public DefaultTalonFX intakeMotor2 = new DefaultTalonFX(intakeMotorConfig2);

  public Intake_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    intakeMotor1.Duty_Cycle_Output(percentage);
    intakeMotor2.Duty_Cycle_Output(percentage);
  }

  public void setIntakePos_func(double pos) {
    intakeMotor1.PID_Position(pos);
    intakeMotor2.PID_Position(pos);
  }
}
