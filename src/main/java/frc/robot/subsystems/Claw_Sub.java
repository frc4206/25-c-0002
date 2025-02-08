// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Claw_Sub extends SubsystemBase {
    /** Creates a new ClawSub. */
    DefaultTalonFX.Config clawMotorConfig1 = new DefaultTalonFX.Config("Claw1Motor.toml");
    DigitalInput clawBeamBreak = new DigitalInput(2);
    public Config clawConfig = new Config("Claw.toml"); 

    public class Config extends LoadableConfig {
        public double intakePercent; 
        public double outtakePercent; 

        public Config(String filename) {

            super.load(this, filename);
            LoadableConfig.print(this);
        }
    }

    public DefaultTalonFX clawMotor1 = new DefaultTalonFX(clawMotorConfig1);

    public Claw_Sub() {
        clawMotor1.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setPercentage_func(double percentage) {
        if (clawBeamBreak.get() != true) {
            clawMotor1.Duty_Cycle_Output(0);
            System.out.println("stopping End Effector");
        } else {
            clawMotor1.Duty_Cycle_Output(percentage);
        }
    }

    public void setPercentageOuttake_func(double percentage) {
        clawMotor1.Duty_Cycle_Output(percentage);
    }

    // public void setPercentageOverride_func(double )
}
