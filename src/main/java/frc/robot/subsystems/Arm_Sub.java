// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class Arm_Sub extends SubsystemBase {
  /** Creates a new armSub. */

  /* Configs */
  ConfigTalonFX.Config armMotorConfig1 = new ConfigTalonFX.Config("Arm1Motor.toml");
  ConfigTalonFX.Config armMotorConfig2 = new ConfigTalonFX.Config("Arm2Motor.toml");
  public Config armConfig;

  /* Motors */
  public TalonFX armMotor1 = new TalonFX(armMotorConfig1.canID, "Default Name");
  public TalonFX armMotor2 = new TalonFX(armMotorConfig2.canID, "Default Name");

  ConfigTalonFX armMotorApply = new ConfigTalonFX(armMotorConfig1, armMotor1);
  ConfigTalonFX armMotor2Apply = new ConfigTalonFX(armMotorConfig1, armMotor2);



  TalonFXConfiguration ltalonConfigs = new TalonFXConfiguration();

  /* Sensors */
  CANcoder armCANCoder;
  DigitalInput armHallSensor;

  // TODO:put in proper values in the tomls and check if they make sense for the
  // subsystem, the filler values will break something if unchanged
  public static class Config extends LoadableConfig {

    /* IDs and Ports */
    public int canCoderID;
    public int limitSwitchPort;

    /* Positions */
    public double stowPosition;
    public double sourceIntakePosition;
    public double l2ScoringPosition;
    public double l3ScoringPosition;
    public double l4ScoringPosition;

    public double maxExtenstion;

    /* Misc. */
    public boolean followerOpposeMaster;

    public Config(String filename) {
      super.load(this, filename);
      // LoadableConfig.print(this);
    }

  }

  public Arm_Sub(Arm_Sub.Config arm_Config) {
    this.armConfig = arm_Config;
    armCANCoder = new CANcoder(armConfig.canCoderID, "Default Name");
    armHallSensor = new DigitalInput(armConfig.limitSwitchPort);

    // LoadableConfig.print(armConfig);
    // LoadableConfig.print(armMotorConfig1);

    // armMotorApply.talonConfigs.Feedback.FeedbackRemoteSensorID = armCANCoder.getDeviceID();
    // armMotorApply.talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // armMotorApply.talonConfigs.Feedback.RotorToSensorRatio = 45;
    // armMotorApply.talonConfigs.Feedback.SensorToMechanismRatio = 1;

    // armMotor2Apply.talonConfigs.Feedback.FeedbackRemoteSensorID = armCANCoder.getDeviceID();
    // armMotor2Apply.talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // armMotor2Apply.talonConfigs.Feedback.RotorToSensorRatio = 45;
    // armMotor2Apply.talonConfigs.Feedback.SensorToMechanismRatio = 1;

    // armMotor2Apply.talonConfigs.Feedback.FeedbackRemoteSensorID = armCANCoder.getDeviceID();
    // armMotor2Apply.talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    // armMotor2Apply.talonConfigs.Feedback.RotorToSensorRatio = 45;
    // armMotor2Apply.talonConfigs.Feedback.SensorToMechanismRatio = 1;

    var mc = new MotorOutputConfigs();
    mc.Inverted = InvertedValue.Clockwise_Positive;
    
    
    armMotorApply.applyConfigs();

    armMotor2Apply.applyConfigs();

    armMotor2Apply.setSlot0(armMotorConfig1.slot0);
    armMotor2Apply.applyConfigs();
    armMotor2.getConfigurator().apply(mc);

    // var request = new Follower(armMotorConfig1.canID, arm_Config.followerOpposeMaster);
    // request.UpdateFreqHz = 50;
    // armMotor2.setControl(request);
    

    /* 
     * This is where we are actually setting the motor RN
     */
    
    // ltalonConfigs.Slot0 = new Slot0Configs().withKP(armMotorConfig1.slot0.kp)
    //     .withKI(armMotorConfig1.slot0.ki)
    //     .withKD(armMotorConfig1.slot0.kd);
    // ltalonConfigs.Feedback.FeedbackRemoteSensorID = armCANCoder.getDeviceID();
    // // ltalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // // ltalonConfigs.Feedback.RotorToSensorRatio = 45;
    // // ltalonConfigs.Feedback.SensorToMechanismRatio = 1;
    // // armMotor1.setInverted(armMotorConfig1.inverted);
    // if (armMotorConfig1.isBreakMode) {
    //   armMotor1.setNeutralMode(NeutralModeValue.Brake);
    // } else {
    //   armMotor1.setNeutralMode(NeutralModeValue.Coast);
    // }
    // armMotor1.getConfigurator().apply(ltalonConfigs);
    // setArms();

    armMotor1.setPosition(armCANCoder.getAbsolutePosition().getValueAsDouble());
    armMotor2.setPosition(armCANCoder.getAbsolutePosition().getValueAsDouble());
    
  }

  public void setPercentage_func(double percentage) {
    armMotor1.setControl(new DutyCycleOut(percentage));
    armMotor2.setControl(new DutyCycleOut(percentage));
  }

  public void setArms() {
    armMotor1.setPosition(0);
    armMotor2.setPosition(0);
  }

  public void setArm2() {
    // armMotor2.setPosition(armConfig.maxExtenstion);
  }

  public void setArmAngle_func(double pos) {
    armMotor1.setPosition(armCANCoder.getAbsolutePosition().getValueAsDouble());
    armMotor2.setPosition(armCANCoder.getAbsolutePosition().getValueAsDouble());

    armMotor1.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
    armMotor2.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var fx_pos = armMotor1.getPosition();
    fx_pos.refresh();

    var fx2_pos = armMotor2.getPosition();
    fx2_pos.refresh();

    // var cc_pos = armCANCoder.getAbsolutePosition();
    // cc_pos.refresh();

    SmartDashboard.putNumber("arm position", fx_pos.getValueAsDouble());
    SmartDashboard.putNumber("arm2 position", fx2_pos.getValueAsDouble());

    // armMotor2.setPosition(fx_pos.getValueAsDouble());
    // SmartDashboard.putNumber("can coder position", armCANCoder.getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("can coder adjusted position", armCANCoder.getAbsolutePosition().getValueAsDouble() * 45);
    // armMotor1.getConfigurator().refresh(ltalonConfigs);
  }
}
