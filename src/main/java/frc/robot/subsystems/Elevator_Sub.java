// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Elevator_Sub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  DefaultTalonFX.Config elevatorMotorConfig1 = new DefaultTalonFX.Config("Elevator1Motor.toml");
  DefaultTalonFX.Config elevatorMotorConfig2 = new DefaultTalonFX.Config("Elevator2Motor.toml");
  DigitalInput elevatorHallSensor1 = new DigitalInput(4);
  DigitalInput elevatorHallSensor2 = new DigitalInput(5);

  public class  Config  extends LoadableConfig {
    public double kHomePosition;

    public double elevatorL1Pos;
    public double elevatorL2Pos;
    public double elevatorL3Pos;
    public double elevatorL4Pos;

    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX elevatorMotor1 = new DefaultTalonFX(elevatorMotorConfig1);
  public DefaultTalonFX elevatorMotor2 = new DefaultTalonFX(elevatorMotorConfig2);

  public Elevator_Sub() {}

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
