// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class Claw_Sub extends SubsystemBase {
    /** Creates a new ClawSub. */

    /*Configs */
    ConfigTalonFX.Config clawMotorConfig1 = new ConfigTalonFX.Config("Claw1Motor.toml");
    public Config clawConfig;

    /*Motors */
    public TalonFX clawMotor1 = new TalonFX(clawMotorConfig1.canID);

    /*Sensors */
    DigitalInput clawBeamBreak;


    public static class Config extends LoadableConfig {

        /*IDs and Ports */
        public int beamBreakPort;
        
        /*Misc. */
        public double intakePercent; 
        public double outtakePercent;

        public Config(String filename) {

            super.load(this, filename);
            LoadableConfig.print(this);
        }
    }


    public Claw_Sub(Config clawConfig) {
        this.clawConfig = clawConfig;
        clawBeamBreak = new DigitalInput(clawConfig.beamBreakPort);
    }

    public void setPercentage_func(double percentage) {
        clawMotor1.setControl(new DutyCycleOut(percentage));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Beam break claw", clawBeamBreak.get());
    }
}
