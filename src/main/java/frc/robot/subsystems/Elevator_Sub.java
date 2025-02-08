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
  DefaultTalonFX.Config elevatorMotorConfig1 = new DefaultTalonFX.Config("elevator1Cfg");
  DefaultTalonFX.Config elevatorMotorConfig2 = new DefaultTalonFX.Config("elevator2Cfg");
  DigitalInput elevatorHallSensor1 = new DigitalInput(4);
  DigitalInput elevatorHallSensor2 = new DigitalInput(5);
  public Config elevatorConfig = new Config("Elevator");
  public Config elevatorMotorConfig = new Config("Elevator1Motor.toml");

  public class  Config  extends LoadableConfig {
    public double stowPosition; 
    public double sourceIntakePosition; 
    public double l1ScoringPosition; 
    public double l2ScoringPosition; 
    public double l3ScoringPosition; 
    public double l4ScoringPosition; 

    public int canID; 

    public Config(String filename){
      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX elevatorMotor1 = new DefaultTalonFX(elevatorMotorConfig1);
  public DefaultTalonFX elevatorMotor2 = new DefaultTalonFX(elevatorMotorConfig2);

  public Elevator_Sub(Elevator_Sub.Config elevator_Motor_Config) {
    elevatorMotorConfig = elevator_Motor_Config; 
    elevatorMotor2.motor.setControl(new Follower(elevator_Motor_Config.canID, false));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentage_func(double percentage) {
    elevatorMotor1.Duty_Cycle_Output(percentage);
    elevatorMotor2.Duty_Cycle_Output(percentage);
  }

  public void setElevatorPos_func(double pos) {
    elevatorMotor1.PID_Position(pos);
    elevatorMotor2.PID_Position(pos);
  }
}
