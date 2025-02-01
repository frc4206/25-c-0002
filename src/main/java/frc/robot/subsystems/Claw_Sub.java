// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Claw_Sub extends SubsystemBase {
  /** Creates a new ClawSub. */
  DefaultTalonFX.Config clawMotorConfig1 = new DefaultTalonFX.Config("claw1Cfg");
  DefaultTalonFX.Config clawMotorConfig2 = new DefaultTalonFX.Config("claw2Cfg");
  DigitalInput beamBreak1 = new DigitalInput(2);

  public class  Config  extends LoadableConfig {
    public double kHomePosition;

    public double ClawL1Pos;
    public double ClawL2Pos;
    public double ClawL3Pos;
    public double ClawL4Pos;

    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX clawMotor1 = new DefaultTalonFX(clawMotorConfig1);
  public DefaultTalonFX clawMotor2 = new DefaultTalonFX(clawMotorConfig2);

  public Claw_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    clawMotor1.Duty_Cycle_Output(percentage);
    clawMotor2.Duty_Cycle_Output(percentage);
  }

  public void reefTroughAngle_func(double pos) {
    clawMotor1.PID_Position(pos);
    clawMotor2.PID_Position(pos);
  }

  public void reefBranchesAngle_func(double pos) {
    clawMotor1.PID_Position(pos);
    clawMotor2.PID_Position(pos);
  }

  public void reefHighestAngle_func(double pos) {
    clawMotor1.PID_Position(pos);
    clawMotor2.PID_Position(pos);
  }

  public void intakeAcceptAngle_func(double pos) {
    clawMotor1.PID_Position(pos);
    clawMotor2.PID_Position(pos);
  }
}
