// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class Intake_Sub extends SubsystemBase {
  /** Creates a new intakeSub. */
  /* Configs */
  ConfigTalonFX.Config intakeMotorRollersConfig = new ConfigTalonFX.Config("IntakeMotorRollers.toml");
  ConfigTalonFX.Config intakeMotorPivotConfig = new ConfigTalonFX.Config("IntakeMotorPivot.toml");

  public Config intakeConfig;

  /* Motors */
  public TalonFX intakeMotorRollers = new TalonFX(intakeMotorRollersConfig.canID);
  public TalonFX intakeMotorPivot = new TalonFX(intakeMotorPivotConfig.canID);

  ConfigTalonFX intakePivotCFGapply = new ConfigTalonFX(intakeMotorPivotConfig, intakeMotorPivot);

  /* Sensors */
  DigitalInput intakeHallSensor;
  DigitalInput intakeBeamBreak;

  public static class Config extends LoadableConfig {

    /* IDs and Ports */
    public int HalllimitSwitch;
    public int beamBreakPort;

    /* Positions */
    public double stowPosition;
    public double l1ScoringPosition;
    public double intakePosition;

    /* Misc. */
    public double intakePercent;
    public double outtakePercent;

    public Config(String filename) {

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public Intake_Sub(Config intakeConfig) {
    this.intakeConfig = intakeConfig;
    intakeHallSensor = new DigitalInput(intakeConfig.HalllimitSwitch);
    intakeBeamBreak = new DigitalInput(intakeConfig.beamBreakPort);

    intakePivotCFGapply.setSlot0(intakeMotorPivotConfig.slot0);
    intakePivotCFGapply.applyConfigs();

  }

  public void setPercentage_func(double percentage) {
    intakeMotorRollers.setControl(new DutyCycleOut(percentage));
  }

  public void setIntakePos_func(double pos) {
    intakeMotorPivot.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!intakeHallSensor.get()) {
      intakeMotorPivot.setPosition(0);
    }
    
  }
}
