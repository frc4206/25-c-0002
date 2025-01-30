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

public class ArmSub extends SubsystemBase {
  /** Creates a new armSub. */
  DefaultTalonFX.Config armMotorConfig1 = new DefaultTalonFX.Config("arm1Cfg");
  DefaultTalonFX.Config armMotorConfig2 = new DefaultTalonFX.Config("arm2Cfg");
  CANcoder armCCoder = new CANcoder(3);
  DigitalInput armHallSensor = new DigitalInput(1);

  public DefaultTalonFX armMotor1 = new DefaultTalonFX(armMotorConfig1);
  public DefaultTalonFX armMotor2 = new DefaultTalonFX(armMotorConfig2);

  //TODO:put in proper values in the tomls and check if they make sense for the subsystem, the filler values will break something if unchanged
  public class  Config  extends LoadableConfig {
    public double kHomePosition;

    public double armL1Pos;
    public double armL2Pos;
    public double armL3Pos;
    public double armL4Pos;


    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  

  public ArmSub(Config cfg) {
      armMotor1.Enable_Sim();
      armMotor2.Enable_Sim();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public void setPercentage_func(double percentage) {
        //TODO: make sure one of these doesn't need to be inverted, double check all motors
        armMotor1.Duty_Cycle_Output(percentage);
        armMotor2.Duty_Cycle_Output(percentage);
  }

  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      super.simulationPeriodic();
      armMotor1.Update_Sim();
      armMotor2.Update_Sim();
  }

  //TODO:also default pos??
  public void intakeAcceptAngle_func(double pos) {
    armMotor1.PID_Position(pos);
    armMotor2.PID_Position(pos);
  }

  public void reefTroughAngle_func(double pos) {
    armMotor1.PID_Position(pos);
    armMotor2.PID_Position(pos);
  }
  public void reefHighestAngle_func(double pos) {
    armMotor1.PID_Position(pos);
    armMotor2.PID_Position(pos);
  }
}
