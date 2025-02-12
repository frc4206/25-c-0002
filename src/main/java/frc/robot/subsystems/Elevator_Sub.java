// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Elevator_Sub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  /*Configs */
  DefaultTalonFX.Config elevatorMotorConfig1 = new DefaultTalonFX.Config("Elevator1Motor.toml");
  DefaultTalonFX.Config elevatorMotorConfig2 = new DefaultTalonFX.Config("Elevator2Motor.toml");
  public Config elevatorConfig = new Config("Elevator");

  /*Motors */
  public DefaultTalonFX elevatorMotor1 = new DefaultTalonFX(elevatorMotorConfig1);
  public DefaultTalonFX elevatorMotor2 = new DefaultTalonFX(elevatorMotorConfig2);

  /*Sensors */
  DigitalInput elevatorHallSensor1 = new DigitalInput(elevatorConfig.limitSwitch1Port);
  DigitalInput elevatorHallSensor2 = new DigitalInput(elevatorConfig.limitSwitch2Port);

  public class  Config  extends LoadableConfig {

    /* IDs and Ports */
    public int limitSwitch1Port;
    public int limitSwitch2Port;

    /*Positions */
    public double stowPosition; 
    public double sourceIntakePosition; 
    public double l1ScoringPosition; 
    public double l2ScoringPosition; 
    public double l3ScoringPosition; 
    public double l4ScoringPosition; 

    public Config(String filename){
      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }


  public Elevator_Sub(Elevator_Sub.Config elevator_Motor_Config) {
    elevatorConfig = elevator_Motor_Config; 
    elevatorMotor2.motor.setControl(new Follower(elevatorMotorConfig1.canID, false));
  }

  public void setPercentage_func(double percentage) {
    elevatorMotor1.Duty_Cycle_Output(percentage);
  }

  public void setElevatorPos_func(double pos) {
    elevatorMotor1.PID_Position(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
