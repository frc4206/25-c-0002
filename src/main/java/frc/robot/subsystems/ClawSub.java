// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class ClawSub extends SubsystemBase {
  /** Creates a new ClawSub. */
  DefaultTalonFX.Config clawMotorConfig1 = new DefaultTalonFX.Config("claw1Cfg");
  DefaultTalonFX.Config clawMotorConfig2 = new DefaultTalonFX.Config("claw2Cfg");
  DigitalInput beamBreak1 = new DigitalInput(2);

  public class  Config  extends LoadableConfig {
    public double kHomePosition;
    public double kCruiseVelocity;
    public double kAcceleration;
    public double kMaxUnitsLimit;
    public double kMinUnitsLimit;
    public double kEnableSupplyCurrentLimit;
    public double kSupplyCurrentLimit;
    public double kSupplyCurrentThreshold;
    public double kSupplyCurrentTimeout;
    public double kMaxForwardOutput;
    public double kMaxReverseOutput;

    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX clawMotor1 = new DefaultTalonFX(clawMotorConfig1);
  public DefaultTalonFX clawMotor2 = new DefaultTalonFX(clawMotorConfig2);

  public ClawSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    clawMotor1.Duty_Cycle_Output(percentage);
    clawMotor2.Duty_Cycle_Output(percentage);
  }
}
