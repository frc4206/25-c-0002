// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Game_Commands.L1_scoring_Com;
import frc.robot.commands.Game_Commands.L2_scoring_Com;
import frc.robot.commands.Game_Commands.L3_scoring_Com;
import frc.robot.commands.Game_Commands.L4_scoring_Com;
import frc.robot.commands.PID_Com.Arm_PID_Com;
import frc.robot.commands.PID_Com.Intake_PID_Com;
import frc.robot.commands.Percent_Com.ClawPercent_Com;
import frc.robot.commands.Percent_Com.IntakePercent_Com;
import frc.robot.common.GameStateCurrentLimiter;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Climber_Sub;
import frc.robot.subsystems.Elevator_Sub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake_Sub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber_Sub.Config m_climbercfg = new Climber_Sub.Config("Climber.toml");


  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Arm_Sub m_Arm_Sub = new Arm_Sub();
  private final Claw_Sub m_Claw_Sub = new Claw_Sub();
  private final Climber_Sub m_Climber_Sub = new Climber_Sub();
  private final Elevator_Sub m_Elevator_Sub = new Elevator_Sub();
  private final Intake_Sub m_Intake_Sub = new Intake_Sub();
  private final GameStateCurrentLimiter m_GameStateCurrentLimiter = new GameStateCurrentLimiter(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_GameStateCurrentLimiter.AddSubsystem(m_climbercfg.name, m_Climber_Sub.m_climberList);
    m_GameStateCurrentLimiter.PopulateSubsystemLimits(m_climbercfg.name, m_Climber_Sub.currentLimitList);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(new InstantCommand(() -> System.out.println("value " + m_ClimberSub.climberMotor1.motor.getSupplyCurrent())));
    // m_driverController.a().onTrue(new InstantCommand(() -> m_GameStateCurrentLimiter.SetState(GameStateCurrentLimiter.State.SHOOT)));
    // m_driverController.x().onTrue(new InstantCommand(() -> m_GameStateCurrentLimiter.SetState(GameStateCurrentLimiter.State.CLIMB)));
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.a().whileTrue(new ClawPercent_Com(m_Claw_Sub, 0.2));
    m_driverController.b().whileTrue(new ClawPercent_Com(m_Claw_Sub, -0.5));

    m_driverController.x().whileTrue(new Arm_PID_Com(m_Arm_Sub, 0));

    // L4 scoring command button line,TODO:change it off of D-pad
    //TODO: test if this works, it should in theory
    m_driverController.pov(0).onTrue(new L4_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)
          .andThen(new ClawPercent_Com(m_Claw_Sub, 0))
          .andThen(new L4_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)));
    
    // L3 scoring command button line
    m_driverController.pov(270).onTrue(new L3_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)
          .andThen(new ClawPercent_Com(m_Claw_Sub, 0))
          .andThen(new L3_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)));

    // L2 scoring command button line
    m_driverController.pov(90).onTrue(new L2_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)
          .andThen(new ClawPercent_Com(m_Claw_Sub, 0))
          .andThen(new L2_scoring_Com(m_Arm_Sub, m_Elevator_Sub, 0, 0)));

    // L1 scoring command button line
    m_driverController.pov(180).onTrue(new L1_scoring_Com(m_Intake_Sub, 0)
          .andThen(new ClawPercent_Com(m_Claw_Sub, 0))
          .andThen(new L1_scoring_Com(m_Intake_Sub, 0)));

    m_driverController.pov(45).onTrue(new Intake_PID_Com(m_Intake_Sub, 0)
          .andThen(new IntakePercent_Com(m_Intake_Sub, 0))
          .andThen(new Intake_PID_Com(m_Intake_Sub, 0)));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}