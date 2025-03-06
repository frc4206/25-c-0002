// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class Elevator_Sub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  /* Configs */
  ConfigTalonFX.Config elevatorMotorConfig1 = new ConfigTalonFX.Config("Elevator1Motor.toml");
  ConfigTalonFX.Config elevatorMotorConfig2 = new ConfigTalonFX.Config("Elevator2Motor.toml");
  public Config elevatorConfig;

  /* Motors */
  public TalonFX elevatorMotor1 = new TalonFX(elevatorMotorConfig1.canID);
  public TalonFX elevatorMotor2 = new TalonFX(elevatorMotorConfig2.canID);

  ConfigTalonFX elevatorConfigApply = new ConfigTalonFX(elevatorMotorConfig1, elevatorMotor1);

  /* Sensors */
  DigitalInput elevatorHallSensor1;
  DigitalInput elevatorHallSensor2;

  public static class Config extends LoadableConfig {

    /* IDs and Ports */
    public int limitSwitch1Port;
    public int limitSwitch2Port;

    /* Positions */
    public double maxExtension;
    public double sourceIntakePosition;
    public double l1ScoringPosition;
    public double l2ScoringPosition;
    public double l3ScoringPosition;
    public double l4ScoringPosition;

    /*Misc. */
    public boolean followerOpposeMaster;

    public Config(String filename) {
      super.load(this, filename);
      // LoadableConfig.print(this);
    }
  }

  public Elevator_Sub(Elevator_Sub.Config elevator_Motor_Config) {
    elevatorConfig = elevator_Motor_Config;
    elevatorHallSensor1 = new DigitalInput(elevatorConfig.limitSwitch1Port);
    elevatorHallSensor2 = new DigitalInput(elevatorConfig.limitSwitch2Port);

    elevatorConfigApply.setSlot0(elevatorMotorConfig1.slot0);
    elevatorConfigApply.applyConfigs();

    
    elevatorMotor2.setControl(new Follower(elevatorMotorConfig1.canID, elevatorConfig.followerOpposeMaster));
  }

  public void setPercentage_func(double percentage) {
    elevatorMotor1.setControl(new DutyCycleOut(percentage));
  }

  public void setElevatorPos_func(double pos) {
    elevatorMotor1.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!elevatorHallSensor1.get()) {
      // elevatorMotor1.setPosition(0);
    }
    if (!elevatorHallSensor2.get()) {
      elevatorMotor1.setPosition(elevatorConfig.maxExtension);
    }

    // SmartDashboard.putBoolean("bottom break", elevatorHallSensor1.get());
    // SmartDashboard.putBoolean("top break", elevatorHallSensor2.get());
    SmartDashboard.putNumber("elevator position", elevatorMotor1.getPosition().getValueAsDouble());
  }
}
