// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;

public class Claw_Sub extends SubsystemBase {
    public enum ClawState {
        NEUTRAL,
        INTAKING,
        DETECTED,
        EXHAUSTING
    }

    private static ClawState claw_state = ClawState.NEUTRAL;

    AsynchronousInterrupt beam_break_interrupt;
    BiConsumer<Boolean, Boolean> trigger;

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
            // LoadableConfig.print(this);
        }
    }

    public Claw_Sub(Config clawConfig) {
        this.clawConfig = clawConfig;
        clawBeamBreak = new DigitalInput(clawConfig.beamBreakPort);
        setupClawBeamBreakInterrupt();
    }

    public void setupClawBeamBreakInterrupt() {
        trigger = new BiConsumer<Boolean, Boolean>() {
            @Override
            public void accept(Boolean rise, Boolean fall) {
                if(rise){
                    claw_state = ClawState.NEUTRAL;
                    System.out.println("Detected a rising edge!");
                    return;
                }

                if (fall) {
                    System.out.print("Claw falling edge!");
                    setPercentage_func(0);
                    claw_state = ClawState.DETECTED;
                    return;
                }
            }
        };
        beam_break_interrupt = new AsynchronousInterrupt(clawBeamBreak, trigger);
        beam_break_interrupt.setInterruptEdges(true, true);
        beam_break_interrupt.enable();
    }

    public void setPercentage_func(double percentage) {
        clawMotor1.setControl(new DutyCycleOut(percentage));
    }

    public ClawState getClawState() {
        return claw_state;
    }

    public void setClawState(ClawState newState) {
        claw_state = newState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Beam break claw", clawBeamBreak.get());
        // System.out.println("\n\nClaw state -->>>> " + claw_state + "\n\n");
        // if (claw_state == ClawState.INTAKING) {
        //     setPercentage_func(1);
        // }
        // switch (claw_state) {
        //     case INTAKING:
        //         //setPercentage_func(clawConfig.intakePercent);
        //         break;
        //     case DETECTED:
                
        //         break;
        //     default:
        //         break;
        // }
    }
}
