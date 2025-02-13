// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import org.opencv.features2d.BFMatcher;
import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.common.DefaultTalonFX;

public class Claw_Sub extends SubsystemBase {

    enum ClawState {
        NEUTRAL,
        INTAKING,
        DETECTED,
        EXHAUSTING
    }

    private ClawState claw_state = ClawState.NEUTRAL;

    AsynchronousInterrupt beam_break_interrupt;

    private XboxController controller;
    private Thread clawThread;

    /** Creates a new ClawSub. */
    DefaultTalonFX.Config clawMotorConfig1 = new DefaultTalonFX.Config("Claw1Motor.toml");// TODO:change can Id back to
                                                                                          // 11

    DigitalInput clawBeamBreak = new DigitalInput(9);

    public DefaultTalonFX clawMotor1 = new DefaultTalonFX(clawMotorConfig1);

    public Claw_Sub(XboxController controller) {
        clawMotor1.motor.setNeutralMode(NeutralModeValue.Brake);
        this.controller = controller;

        setupClawBeamBreakInterrupt();
    }

    public void setupClawBeamBreakInterrupt() {

        BiConsumer<Boolean, Boolean> trigger = new BiConsumer<Boolean, Boolean>() {
            @Override
            public void accept(Boolean t, Boolean u) {
                if (u) {
                    clawMotor1.motor.setControl(new DutyCycleOut(0.0d));
                    claw_state = ClawState.DETECTED;
                }

                if (t) {
                    claw_state = ClawState.NEUTRAL;
                }
            }
        };

        beam_break_interrupt = new AsynchronousInterrupt(clawBeamBreak, trigger);
        beam_break_interrupt.setInterruptEdges(true, true);
        beam_break_interrupt.enable();
    }

    public static long ridiculousFunction(int n) {
        if (n <= 1) {
            return n;
        }
        return ridiculousFunction(n - 1) + ridiculousFunction(n - 2); // Recursion to
        // make it inefficient
    }

    @Override
    public void periodic() {

        if (claw_state == ClawState.DETECTED)
            return;

        if (controller.getBButton()) {
            clawMotor1.motor.setControl(new DutyCycleOut(-1.0d));
            claw_state = ClawState.INTAKING;
        } else {
            clawMotor1.motor.setControl(new DutyCycleOut(0.0d));
            claw_state = ClawState.NEUTRAL;
        }
    }
}
