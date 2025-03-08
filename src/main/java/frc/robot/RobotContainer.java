// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetClawStateCommand;
import frc.robot.commands.Swerve_PID;
import frc.robot.commands.Game_Commands.Coral_Intake_Com;
import frc.robot.commands.Game_Commands.L1_scoring_Com;
import frc.robot.commands.Game_Commands.L2_scoring_Com;
import frc.robot.commands.Game_Commands.L3_scoring_Com;
import frc.robot.commands.Game_Commands.L4_scoring_Com;
import frc.robot.commands.PID_Commands.Arm_PID_Com;
import frc.robot.commands.PID_Commands.Elevator_PID_Com;
import frc.robot.commands.PID_Commands.Intake_PID_Com;
import frc.robot.commands.Percent_Commands.ArmPercent_Com;
import frc.robot.commands.Percent_Commands.ClawPercent_Com;
import frc.robot.commands.Percent_Commands.ClimberPercent_Com;
import frc.robot.commands.Percent_Commands.ElevatorPercent_Com;
import frc.robot.commands.Percent_Commands.IntakePercent_Com;
import frc.robot.commands.Test_Commands.ArmJoystick_Com;
import frc.robot.commands.Test_Commands.ClawJoystick_Com;
import frc.robot.commands.Test_Commands.ClimberJoystick_Com;
import frc.robot.commands.Test_Commands.ElevatorJoystick_Com;
import frc.robot.commands.Test_Commands.IntakeJoystick_Com;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm_Sub;
import frc.robot.subsystems.Claw_Sub;
import frc.robot.subsystems.Climber_Sub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator_Sub;
import frc.robot.subsystems.Intake_Sub;
import frc.robot.subsystems.Claw_Sub.ClawState;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Arm_Sub.Config m_armConfig = new Arm_Sub.Config("Arm.toml");
  public final Claw_Sub.Config m_clawConfig = new Claw_Sub.Config("Claw.toml");
  public final Climber_Sub.Config m_climberCfg = new Climber_Sub.Config("Climber.toml");
  public final Elevator_Sub.Config m_elevatorCfg = new Elevator_Sub.Config("Elevator.toml");
  public final Intake_Sub.Config m_intakeCfg = new Intake_Sub.Config("Intake.toml");

  final Arm_Sub m_arm = new Arm_Sub(m_armConfig);
  final Claw_Sub m_claw = new Claw_Sub(m_clawConfig);
  final Climber_Sub m_climber = new Climber_Sub(m_climberCfg);
  final Elevator_Sub m_elevator = new Elevator_Sub(m_elevatorCfg);
  final Intake_Sub m_intake = new Intake_Sub(m_intakeCfg);

  private final CommandXboxController m_operatorController = new CommandXboxController(1); 
  //private final CommandXboxController m_armController = new CommandXboxController(1);
  // private final CommandXboxController m_clawController = new CommandXboxController(2);
  // private final CommandXboxController m_climberController = new CommandXboxController(3);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(4);
  // private final CommandXboxController m_intakeController = new CommandXboxController(5);

  

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  TunedJoystick tj = new TunedJoystick(m_driverController.getHID())
    .setDeadzone(0.1)
    .useResponseCurve(ResponseCurve.QUADRATIC)
    .setPeriodMilliseconds(10);

  /* SWERVE */
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // PATHPLANNER COMMANDS
    // Arm Commands
    // NamedCommands.registerCommand("L4Arm", new Arm_PID_Com(m_arm, m_arm.armConfig.l4ScoringPosition));
    // NamedCommands.registerCommand("L3Arm", new Arm_PID_Com(m_arm, m_arm.armConfig.l3ScoringPosition));

    // // Elevator Commands
    // NamedCommands.registerCommand("L4Elevator",
    //     new Elevator_PID_Com(m_elevator, m_elevator.elevatorConfig.l4ScoringPosition));
    // NamedCommands.registerCommand("L3Elevator",
    //     new Elevator_PID_Com(m_elevator, m_elevator.elevatorConfig.l3ScoringPosition));
    // NamedCommands.registerCommand("IntakeElevator",
    //     new Elevator_PID_Com(m_elevator, m_elevator.elevatorConfig.sourceIntakePosition));

    // // Claw Commands
    NamedCommands.registerCommand("Score", new InstantCommand(() -> m_claw.clawMotor1.set(1)).withTimeout(.5));
    // NamedCommands.registerCommand("AlgaClaw", new ClawPercent_Com(m_claw, m_claw.clawConfig.intakePercent));
    NamedCommands.registerCommand("Intake", new ClawPercent_Com(m_claw, m_claw.clawConfig.intakePercent).withTimeout(1));

    NamedCommands.registerCommand("L4Score", new L4_scoring_Com(m_arm, m_claw, m_elevator).withTimeout(1));
    NamedCommands.registerCommand("CoralIntake", new Coral_Intake_Com(m_arm, m_claw, m_elevator).withTimeout(1));
    // NamedCommands.registerCommand("RunEndEffector", new ClawPercent_Com(m_claw, m_clawConfig.intakePercent).withTimeout(1));
    NamedCommands.registerCommand("NeutralizeEndEffector", new SetClawStateCommand(m_claw, ClawState.NEUTRAL));

    // Configure the trigger bindings
    configureBindings();

    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith(""))
            : stream);

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-tj.getLeftY() * MaxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
            .withVelocityY(-tj.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-tj.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));

    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    m_driverController.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Joystick commands
    // m_arm.setDefaultCommand(new ArmJoystick_Com(m_arm, m_armController));
    // m_claw.setDefaultCommand(new ClawJoystick_Com(m_claw, m_armController));
    m_climber.setDefaultCommand(new ClimberJoystick_Com(m_climber, m_operatorController));
    // m_elevator.setDefaultCommand(new ElevatorJoystick_Com(m_elevator, m_elevatorController));
    // m_intake.setDefaultCommand(new IntakeJoystick_Com(m_intake,
    // m_intakeController));
    // m_armController.rightBumper().whileTrue(new IntakePercent_Com(m_intake,
    // 0.8));
    // m_armController.leftBumper().whileTrue(new IntakePercent_Com(m_intake,
    // -0.8));
    // Make sure the motors are spinning the correct direction

    // m_armController.a().whileTrue(new ArmPercent_Com(m_arm, 0.1));
    // m_armController.b().whileTrue(new ArmPercent_Com(m_arm, -0.1));
    // m_armController.x().onTrue(new InstantCommand(() ->
    // m_arm.armMotor2.setControl(new DutyCycleOut(0.1))));
    // m_armController.x().onFalse(new InstantCommand(() ->
    // m_arm.armMotor2.setControl(new DutyCycleOut(0))));
    // m_armController.y().onTrue(new InstantCommand(() ->
    // m_arm.armMotor2.setControl(new DutyCycleOut(-0.1))));
    // m_armController.y().onFalse(new InstantCommand(() ->
    // m_arm.armMotor2.setControl(new DutyCycleOut(0))));
    // m_intakeController.a().whileTrue(new Intake_PID_Com(m_intake, m_intakeCfg.stowPosition));
    // m_intakeController.b().whileTrue(new Intake_PID_Com(m_intake, m_intakeCfg.l1ScoringPosition));
    // m_climberController.y().whileTrue(new Intake_PID_Com(m_intake, m_intakeCfg.intakePosition));
    // m_intakeController.x().whileTrue(new Intake_PID_Com(m_intake, m_intakeCfg.algePosition));

    // m_climberController.a().whileTrue(new ClimberPercent_Com(m_climber, 0.1));
    // m_climberController.b().whileTrue(new ClimberPercent_Com(m_climber, -0.1));
    // m_climberController.x().onTrue(new InstantCommand(() ->
    // m_climber.climberMotor2.setControl(new DutyCycleOut(0.1))));
    // m_climberController.x().onFalse(new InstantCommand(() ->
    // m_climber.climberMotor2.setControl(new DutyCycleOut(0))));
    // m_climberController.y().onTrue(new InstantCommand(() ->
    // m_climber.climberMotor2.setControl(new DutyCycleOut(-0.1))));
    // m_climberController.y().onFalse(new InstantCommand(() ->
    // m_climber.climberMotor2.setControl(new DutyCycleOut(0))));

    // m_climberController.a().whileTrue(new ClimberPercent_Com(m_climber, 0.04));
    // m_climberController.b().whileTrue(new InstantCommand(() ->
    // m_climber.m_Servo.setSpeed(-.5)));
    // m_climberController.x().onFalse(new InstantCommand(() ->
    // m_climber.m_Servo.setSpeed(0)));

    // m_elevatorController.a().whileTrue(new ElevatorPercent_Com(m_elevator, 0.1));
    // m_elevatorController.b().whileTrue(new ElevatorPercent_Com(m_elevator,
    // -0.1));
    // m_elevatorController.x().onTrue(new InstantCommand(() ->
    // m_elevator.elevatorMotor2.setControl(new DutyCycleOut(0.1))));
    // m_elevatorController.x().onFalse(new InstantCommand(() ->
    // m_elevator.elevatorMotor2.setControl(new DutyCycleOut(0))));
    // m_elevatorController.y().onTrue(new InstantCommand(() ->
    // m_elevator.elevatorMotor2.setControl(new DutyCycleOut(-0.1))));
    // m_elevatorController.y().onFalse(new InstantCommand(() ->
    // m_elevator.elevatorMotor2.setControl(new DutyCycleOut(0))));
    // m_elevatorController.a().whileTrue(new Elevator_PID_Com(m_elevator, m_elevatorCfg.l1ScoringPosition));
    // m_armController.y().onTrue(new Elevator_PID_Com(m_elevator,
    // m_elevatorCfg.l4ScoringPosition));
    // m_elevatorController.y().whileTrue(new Elevator_PID_Com(m_elevator, m_elevatorCfg.l4ScoringPosition));

    // m_armController.a().onTrue(new Arm_PID_Com(m_arm, m_armConfig.sourceIntakePosition));
    // m_armController.b().onTrue(new Arm_PID_Com(m_arm, m_armConfig.l2ScoringPosition));
    // m_armController.x().onTrue(new Arm_PID_Com(m_arm, m_armConfig.l3ScoringPosition));
    // m_armController.y().onTrue(new Arm_PID_Com(m_arm, m_armConfig.l4ScoringPosition));

    // m_armController.rightBumper().onTrue(new ClawPercent_Com(m_claw,
    // m_clawConfig.intakePercent));
    // m_armController.leftBumper().onTrue(new InstantCommand(() ->
    // m_claw.clawMotor1.setControl(new DutyCycleOut(0.75))));
    // m_armController.x().onTrue(new InstantCommand(() ->
    // m_claw.clawMotor1.setControl(new DutyCycleOut(0))));

    m_operatorController.rightBumper().onTrue(new Coral_Intake_Com(m_arm, m_claw, m_elevator));
    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_claw.clawMotor1.setControl(new DutyCycleOut(0.65))));


    m_operatorController.pov(0).onTrue(new SetClawStateCommand(m_claw, ClawState.NEUTRAL));
    m_operatorController.pov(90).onTrue(new InstantCommand(() -> m_claw.clawMotor1.setControl(new DutyCycleOut(1))));
    m_operatorController.pov(270).onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.algePosition));
    m_operatorController.pov(180).onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.intakePosition));

    m_operatorController.x().onTrue(new L1_scoring_Com(m_intake)); 
    m_operatorController.a().onTrue(new L2_scoring_Com(m_arm, m_claw, m_elevator));
    m_operatorController.b().onTrue(new L3_scoring_Com(m_arm, m_claw, m_elevator));
    m_operatorController.y().onTrue(new L4_scoring_Com(m_arm, m_claw, m_elevator));

    m_operatorController.rightTrigger().onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.l1ScoringPosition));
    m_operatorController.leftTrigger().onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.stowPosition));
    m_operatorController.rightStick().onTrue(new InstantCommand(() -> m_arm.setArms()));
    
    // m_operatorController.getHID().getRawButton(8).onTrue(new IntakePercent_Com(m_intake, .7));
    JoystickButton back = new JoystickButton(m_operatorController.getHID(), 7);
    JoystickButton start = new JoystickButton(m_operatorController.getHID(), 8);
    back.onTrue(new IntakePercent_Com(m_intake, .7));
    start.onTrue(new IntakePercent_Com(m_intake, -.7));
    back.onFalse(new IntakePercent_Com(m_intake, 0));
    start.onFalse(new IntakePercent_Com(m_intake, 0));

    
    m_driverController.leftBumper().whileTrue(new Swerve_PID(drivetrain, -0.165, MaxSpeed, MaxAngularRate, tj));
    m_driverController.rightBumper().whileTrue(new Swerve_PID(drivetrain, 0.165, MaxSpeed, MaxAngularRate, tj));
;
    // m_intake.setDefaultCommand(new Intake_PID_Com(m_intake, 0));

    // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(0.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}