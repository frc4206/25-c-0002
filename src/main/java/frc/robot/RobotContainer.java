// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoLineUp;
import frc.robot.commands.SetClawStateCommand;
import frc.robot.commands.Swerve_PID;
import frc.robot.commands.moveinauto;
import frc.robot.commands.Auto_Commands.Coral_Intake_React_Com;
import frc.robot.commands.Auto_Commands.L4_scoring_React_Com;
import frc.robot.commands.Game_Commands.Coral_Intake_Com;
import frc.robot.commands.Game_Commands.L1_scoring_Com;
import frc.robot.commands.Game_Commands.L2_scoring_Com;
import frc.robot.commands.Game_Commands.L3_scoring_Com;
import frc.robot.commands.Game_Commands.L4_scoring_Com;
import frc.robot.commands.Game_Commands.LineUpPP;
import frc.robot.commands.Game_Commands.SwervePPAlign;
import frc.robot.commands.PID_Commands.Arm_PID_Com;
import frc.robot.commands.PID_Commands.Elevator_MotionMagic_Com;
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
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.SignalLogger;
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

import java.io.IOException;
import java.util.jar.Attributes.Name;

import org.json.simple.parser.ParseException;
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
  private final CommandXboxController m_armController = new CommandXboxController(2);
  // private final CommandXboxController m_clawController = new CommandXboxController(2);
  // private final CommandXboxController m_climberController = new CommandXboxController(3);
  private final CommandXboxController m_elevatorController = new CommandXboxController(4);
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
    NamedCommands.registerCommand("Score", new InstantCommand(() -> m_claw.clawMotor1.set(0.5)).withTimeout(0.5));
    NamedCommands.registerCommand("ScoreReact", new ClawPercent_Com(m_claw, m_clawConfig.outtakePercent));
    // NamedCommands.registerCommand("AlgaClaw", new ClawPercent_Com(m_claw, m_claw.clawConfig.intakePercent));
    NamedCommands.registerCommand("Intake", new ClawPercent_Com(m_claw, m_claw.clawConfig.intakePercent).withTimeout(0.5));

    NamedCommands.registerCommand("L4Score", new L4_scoring_Com(m_arm, m_claw, m_elevator).withTimeout(0.7));
    NamedCommands.registerCommand("L4ScoreReact", new L4_scoring_React_Com(m_arm, m_elevator));
    NamedCommands.registerCommand("CoralIntakeReact", new Coral_Intake_React_Com(m_arm, m_claw, m_elevator));

    NamedCommands.registerCommand("CoralIntake", new Coral_Intake_Com(m_arm, m_claw, m_elevator).withTimeout(0.5));
    // NamedCommands.registerCommand("RunEndEffector", new ClawPercent_Com(m_claw, m_clawConfig.intakePercent).withTimeout(1));
    NamedCommands.registerCommand("NeutralizeEndEffector", new SetClawStateCommand(m_claw, ClawState.EXHAUSTING).withTimeout(1));
    //new Swerve_PID(drivetrain, -0.165 - 0.0127, MaxSpeed, MaxAngularRate, tj)
    NamedCommands.registerCommand("LeftLineUp", new AutoLineUp(drivetrain, -0.165, MaxSpeed, MaxAngularRate, tj));
    NamedCommands.registerCommand("RightLineUp", new AutoLineUp(drivetrain, 0.165, MaxSpeed, MaxAngularRate, tj));
    NamedCommands.registerCommand("FloorIntakeUp", new Intake_PID_Com(m_intake, m_intakeCfg.stowPosition));

    int[] twenty2 = {22};
    // NamedCommands.registerCommand("TAG22", new InstantCommand(() -> LimelightHelpers.SetFiducialIDFiltersOverride("limelight-intake", twenty2)));

    NamedCommands.registerCommand("PivotIntake", new Arm_PID_Com(m_arm, m_armConfig.sourceIntakePosition));
    NamedCommands.registerCommand("ElevatorIntake", new Elevator_PID_Com(m_elevator, m_elevatorCfg.sourceIntakePosition));
    NamedCommands.registerCommand("ResetClaw", new SetClawStateCommand(m_claw, ClawState.NEUTRAL).withTimeout(1));


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
    m_driverController.a().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Joystick commands
    m_climber.setDefaultCommand(new ClimberJoystick_Com(m_climber, m_operatorController));

    m_operatorController.rightBumper().onTrue(new Coral_Intake_Com(m_arm, m_claw, m_elevator));
    m_operatorController.leftBumper().whileTrue(new ClawPercent_Com(m_claw, m_clawConfig.outtakePercent));



    m_operatorController.pov(0).onTrue(new SetClawStateCommand(m_claw, ClawState.EXHAUSTING));
    // m_operatorController.pov(90).onTrue(new InstantCommand(() -> m_claw.clawMotor1.setControl(new DutyCycleOut(0))));
    m_operatorController.pov(270).onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.algePosition));
    m_operatorController.pov(180).onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.intakePosition));

    m_operatorController.x().onTrue(new InstantCommand(() -> m_claw.clawMotor1.setControl(new DutyCycleOut(0)))); 
    m_operatorController.a().onTrue(new L2_scoring_Com(m_arm, m_claw, m_elevator));
    m_operatorController.b().onTrue(new L3_scoring_Com(m_arm, m_claw, m_elevator));
    m_operatorController.y().onTrue(new L4_scoring_Com(m_arm, m_claw, m_elevator));

    m_operatorController.rightTrigger().onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.l1ScoringPosition));
    m_operatorController.leftTrigger().onTrue(new Intake_PID_Com(m_intake, m_intakeCfg.stowPosition));
    m_operatorController.rightStick().onTrue(new InstantCommand(() -> m_arm.setArms()));

    // m_operatorController.getHID().getRawButton(8).onTrue(new IntakePercent_Com(m_intake, .7));
    JoystickButton back = new JoystickButton(m_operatorController.getHID(), 7);
    JoystickButton start = new JoystickButton(m_operatorController.getHID(), 8);
    back.onTrue(new IntakePercent_Com(m_intake, .25));
    start.onTrue(new IntakePercent_Com(m_intake, -.7));
    back.onFalse(new IntakePercent_Com(m_intake, 0));
    start.onFalse(new IntakePercent_Com(m_intake, 0));

    
    // m_driverController.leftBumper().whileTrue(new Swerve_PID(drivetrain, -0.165, MaxSpeed, MaxAngularRate, tj));
    // m_driverController.rightBumper().whileTrue(new Swerve_PID(drivetrain, 0.165, MaxSpeed, MaxAngularRate, tj));
    m_driverController.rightBumper().whileTrue(new SwervePPAlign(drivetrain, "R"));
    m_driverController.leftBumper().whileTrue(new SwervePPAlign(drivetrain, "L"));

    //TODO: add the Intake React Command that will terminate it in a deadline command group
    m_driverController.rightTrigger().whileTrue(drivetrain.followPathCommand("RightIntake"));
    m_driverController.leftTrigger().whileTrue(drivetrain.followPathCommand("LeftIntake"));

    m_intake.setDefaultCommand(new Intake_PID_Com(m_intake, 0));


    m_driverController.b().onTrue(new InstantCommand(() -> SignalLogger.start()));
    m_driverController.pov(0).onTrue(new InstantCommand(() -> SignalLogger.stop()));
    // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(0.1)));

    m_elevatorController.a().onTrue(new Elevator_MotionMagic_Com(m_elevator, m_elevatorCfg.l2ScoringPosition));
    m_elevatorController.b().onTrue(new Elevator_MotionMagic_Com(m_elevator, m_elevatorCfg.l3ScoringPosition));
    m_elevatorController.y().onTrue(new Elevator_MotionMagic_Com(m_elevator, m_elevatorCfg.l4ScoringPosition));

    m_armController.a().onTrue(new Arm_PID_Com(m_arm, m_armConfig.l2ScoringPosition));
    m_armController.b().onTrue(new Arm_PID_Com(m_arm, m_armConfig.l4ScoringPosition));
    m_armController.y().onTrue(new Arm_PID_Com(m_arm, m_armConfig.sourceIntakePosition));

    m_armController.rightBumper().onTrue(m_arm.runOnce(() -> m_arm.armMotor1.set(0.2)));
    m_armController.rightBumper().onFalse(m_arm.runOnce(() -> m_arm.armMotor1.set(0)));

    m_armController.leftBumper().onTrue(m_arm.runOnce(() -> m_arm.armMotor1.set(-0.2)));
    m_armController.leftBumper().onFalse(m_arm.runOnce(() -> m_arm.armMotor1.set(0)));
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