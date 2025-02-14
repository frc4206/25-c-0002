// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Arm_Sub extends SubsystemBase {
  /** Creates a new armSub. */

  /*Configs */
  DefaultTalonFX.Config armMotorConfig1 = new DefaultTalonFX.Config("Arm1Motor.toml");
  DefaultTalonFX.Config armMotorConfig2 = new DefaultTalonFX.Config("Arm2Motor.toml");
  public Config armConfig; 

  /*Motors */
  public DefaultTalonFX armMotor1 = new DefaultTalonFX(armMotorConfig1);
  public DefaultTalonFX armMotor2 = new DefaultTalonFX(armMotorConfig2);
  
  /*Sensors */
  CANcoder armCCoder = new CANcoder(armConfig.canCoderID);
  DigitalInput armHallSensor = new DigitalInput(armConfig.limitSwitchPort);

  //TODO:put in proper values in the tomls and check if they make sense for the subsystem, the filler values will break something if unchanged
  public class  Config  extends LoadableConfig {

    /*IDs and Ports */
    public int canCoderID;
    public int limitSwitchPort;

    /*Positions */
    public double stowPosition;
    public double sourceIntakePosition;
    public double l2ScoringPosition;
    public double l3ScoringPosition;
    public double l4ScoringPosition;

    /*Misc. */
    

    public Config(String filename){
      super.load(this, filename);
      LoadableConfig.print(this);
    }

  }

  public Arm_Sub(Arm_Sub.Config arm_Config) {
    this.armConfig = arm_Config; 
    armMotor2.motor.setControl(new Follower(armMotorConfig1.canID, false));
  }

  public void setPercentage_func(double percentage) {
        //TODO: make sure one of these doesn't need to be inverted, double check all motors
        armMotor1.Duty_Cycle_Output(percentage);
  }

  @Override
  public void simulationPeriodic() {
      super.simulationPeriodic();
      armMotor1.Update_Sim();
      armMotor2.Update_Sim();
  }

  public void setArmAngle_func(double pos) {
    armMotor1.PID_Position(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }
}
