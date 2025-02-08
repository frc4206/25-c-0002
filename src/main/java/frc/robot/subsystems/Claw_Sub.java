// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.common.DefaultTalonFX;

public class Claw_Sub extends SubsystemBase {

    private XboxController controller;
    private Thread clawThread;

    /** Creates a new ClawSub. */
    DefaultTalonFX.Config clawMotorConfig1 = new DefaultTalonFX.Config("Claw1Motor.toml");// TODO:change can Id back to
                                                                                          // 11

    DigitalInput clawBeamBreak = new DigitalInput(9);

    public class Config extends LoadableConfig {

        public Config(String filename) {

            super.load(this, filename);
            LoadableConfig.print(this);
        }
    }

    public DefaultTalonFX clawMotor1 = new DefaultTalonFX(clawMotorConfig1);

    public Claw_Sub(XboxController controller) {
        clawMotor1.motor.setNeutralMode(NeutralModeValue.Brake);
        this.controller = controller;

        setupClawThread();
    }

    public void clawPeriodic() throws InterruptedException {
        if(clawBeamBreak.get() && controller.getBButton())
        {
            clawMotor1.Duty_Cycle_Output(-1.0);
        } else {
            clawMotor1.Duty_Cycle_Output(0.0);
        }
    }

    public void setupClawThread() {
        clawThread = new Thread() {
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        clawPeriodic();
                        Thread.sleep(5);
                    } catch (InterruptedException v) {
                        System.out.println(v);
                    }
                }
            }
        };

        clawThread.start();
    }

    public static long ridiculousFunction(int n) {
        if (n <= 1) {
          return n;
        }
        return ridiculousFunction(n - 1) + ridiculousFunction(n - 2); // Recursion to make it inefficient
      }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Claw_Sub.ridiculousFunction(35);
    }

    public void setPercentage_func(double percentage) {
        if (clawBeamBreak.get() != true) {
            clawMotor1.Duty_Cycle_Output(0);
            System.out.print("stopping End Effector");
        } else {
            clawMotor1.Duty_Cycle_Output(percentage);
        }
    }

    // public void setPercentageOverride_func(double )
}
