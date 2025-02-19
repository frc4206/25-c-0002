// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber_Sub;

public class Climber_Default_Com extends Command {
	/** Creates a new Climber_Default_Command. */
	public int engageServoPos = 0; // The position the servo is in when it is engaged 
	public int disengageServoPos = 0; // The position the servo is in when it is disengaged 
	private boolean servoDisengaged = false; // The state of the motor being engaged or disengeged
	private long startServoTime = 0; // the time from the start that the motor has been running
	private long disengageDuractionMilliseconds = 180; // 0.2 seconds (human reaction time)

	double motor_speed_set = 0.0d;// the speed at which the motor is currently set to
	Climber_Sub m_climberSub;

	public Climber_Default_Com(Climber_Sub climberSub) {
		m_climberSub = climberSub;

		addRequirements(m_climberSub);
		// Makes the climber subsystem required for running this command
	}

	PWM servo;// = new PWM(Constants.Climber.servoLeftID);

	@Override
	public void execute() {
		// This method will be called once per scheduler run
		
		// servo.setBoundsMicroseconds(2500, 2100, 1500, 500, 700);

		if (motor_speed_set < 0.0d) {
			servo.setPulseTimeMicroseconds(this.disengageServoPos);
			servo.setPosition(this.disengageServoPos);
			// start a timer if we are just now pressing the button
			if (!this.servoDisengaged) {
				this.startServoTime = System.currentTimeMillis();
				this.servoDisengaged = true;
			}
		} else {
			// spinning with from pawl
			servo.setPulseTimeMicroseconds(this.engageServoPos);
			servo.setPosition(this.engageServoPos);
			this.servoDisengaged = false;
		}

		long currentTime = System.currentTimeMillis();

		if (currentTime - startServoTime <= this.disengageDuractionMilliseconds && this.servoDisengaged) {
			motor_speed_set = 0.0d;
			// Disables the motor if nothing has been pressed
		}

		// set motor
		// climber_motor.set(motor_speed_set);

	}
}
