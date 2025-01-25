package frc.robot.common;
import javax.swing.plaf.basic.BasicComboBoxUI.FocusHandler;
import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;
import javax.swing.text.StyleContext.SmallAttributeSet;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DefaultTalonFX {
    //-----------------------Set Up-----------------------

    DefaultTalonFX.Config cfg;

    //Creates Motor
    public TalonFX motor;

    //Creates PID Config
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

    //Default Test command
    private InstantCommand defaultCommand = new InstantCommand(() -> System.out.println("Default test command ran on " + cfg.name));

    //Simulation
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final DCMotorSim m_motorSimModel = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.001);
    
    //SmartDashboard Logging
    private boolean LivePIDInit = false;
    private boolean LogDataInit = false;
    private boolean LiveMotionMagicInit = false;

    private double desired_Percent = 0;

    public DefaultTalonFX (DefaultTalonFX.Config cfg) {
        this.cfg = cfg;
        motor = new TalonFX(this.cfg.canID);

        setSlot0(cfg.slot0);
        setSlot1(cfg.slot1);
        setSlot2(cfg.slot2);
    }

    public static class BasicSlot extends LoadableConfig {
        public double kp; // proportional
        public double ki; // integral
        public double kd; // derivative

        public BasicSlot(){};
    }

    public static class Slot extends LoadableConfig {
        public double kp; // proportional
        public double ki; // integral
        public double kd; // derivative
        public double ks; // static feedforward
        public double kv; // velocity feedforward
        public double ka; // acceleration feedforward
    
        public Slot(){};
    }

    //Configuration
    public static final class Config extends LoadableConfig {
        public String name;
        public int canID;
        public boolean inverted;

        public BasicSlot slot0;
        public BasicSlot slot1;
        public BasicSlot slot2;

		public Config(String filename) {
			super.load(this, filename);
			LoadableConfig.print(this);
		}
	}

    //PID Setup
    private double setPoint = 0;

    public void testLoop(){
        TalonFXConfiguration tc = talonConfigs;

        // for(SlotConfigs slot : tc.Slot0)
    }

    public void setSlot0(BasicSlot bs){
        talonConfigs.Slot0.kP = bs.kp;
        talonConfigs.Slot0.kI = bs.ki;
        talonConfigs.Slot0.kD = bs.kd;
    }

    public void setSlot1(BasicSlot bs){
        talonConfigs.Slot1.kP = bs.kp;
        talonConfigs.Slot1.kI = bs.ki;
        talonConfigs.Slot1.kD = bs.kd;
    }

    public void setSlot2(BasicSlot bs){
        talonConfigs.Slot2.kP = bs.kp;
        talonConfigs.Slot2.kI = bs.ki;
        talonConfigs.Slot2.kD = bs.kd;
    }


    public void Set_PID_Slot(int slot, double P, double I, double D) {
        if (slot == 0) {
            talonConfigs.Slot0.kP = P;
            talonConfigs.Slot0.kI = I;
            talonConfigs.Slot0.kD = D;
        } else if (slot == 1) {
            talonConfigs.Slot1.kP = P;
            talonConfigs.Slot1.kI = I;
            talonConfigs.Slot1.kD = D;
        } else if (slot == 2) {
            talonConfigs.Slot2.kP = P;
            talonConfigs.Slot2.kI = I;
            talonConfigs.Slot2.kD = D;
        }
        motor.getConfigurator().apply(talonConfigs);
    }

    // public <T extends ParentConfiguration> void setSlot(T fxSlot, BasicSlot bs){
    //     fxSlot.kP = bs.kp;
    //     fxSlot.kI = bs.ki;
    //     fxSlot.kD = bs.kd;
    // }

    public void Set_SVA_Slot(int slot, double S, double V, double A) {
        if (slot == 0) {
            talonConfigs.Slot0.kS = S;
            talonConfigs.Slot0.kV = V;
            talonConfigs.Slot0.kA = A;
        } else if (slot == 1) {
            talonConfigs.Slot1.kS = S;
            talonConfigs.Slot1.kV = V;
            talonConfigs.Slot1.kA = A;
        } else if (slot == 2) {
            talonConfigs.Slot2.kS = S;
            talonConfigs.Slot2.kV = V;
            talonConfigs.Slot2.kA = A;
        }
        motor.getConfigurator().apply(talonConfigs);
    }

    //Motion Magic Set Up
    public void Motion_Magic_Setup(double cruise_velo, double acceleration, double jerk) {
        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = cruise_velo;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;

        motor.getConfigurator().apply(talonConfigs);
    }

    //-----------------------Motor controls-----------------------
    //Basic controls
    public void Set_Request(ControlRequest request) {
        motor.setControl(request);
    }

    public void Set_Percent_Output(double percent) {
        motor.set(percent);
    }

    //Control Output Types
    public void Duty_Cycle_Output(double percent) {
        final DutyCycleOut m_request = new DutyCycleOut(percent);
        motor.setControl(m_request);
    }

    public void Duty_Cycle(double percent, boolean FOC) {
        final DutyCycleOut m_request = new DutyCycleOut(percent);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request);
    }

    public void Voltage_Output(double voltage) {
        final VoltageOut m_request = new VoltageOut(voltage);
        motor.setControl(m_request);
    }

    public void Voltage_Output(double voltage, boolean FOC) {
        final VoltageOut m_request = new VoltageOut(voltage);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request);
    }

    public void Torque_Current_FOC(double current) {
        final TorqueCurrentFOC m_request = new TorqueCurrentFOC(current);
        motor.setControl(m_request);
    }

    //Basic PID stuff
    public void PID_Position(double Pos) {
        final PositionVoltage m_request = new PositionVoltage(Pos).withSlot(0);
        motor.setControl(m_request);
    }

    public void PID_Position(double Pos, boolean FOC) {
        final PositionVoltage m_request = new PositionVoltage(Pos).withSlot(0);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request);
    }

    public void PID_Velocity(double Velo) {
        final VelocityVoltage m_request = new VelocityVoltage(Velo).withSlot(0);
        motor.setControl(m_request);
    }

    public void PID_Velocity(double Velo, boolean FOC) {
        final VelocityVoltage m_request = new VelocityVoltage(Velo).withSlot(0);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request);
    }

    //Motion Magic
    public void Motion_Magic_Pos(double pos) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        motor.setControl(m_request.withPosition(pos));
    }

    public void Motion_Magic_Pos(double pos, boolean FOC) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request.withPosition(pos));
    }

    public void Motion_Magic_Velo(double velo) {
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        motor.setControl(m_request.withVelocity(velo));
    }
    
    public void Motion_Magic_Velo(double velo, boolean FOC) {
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        m_request.EnableFOC = FOC;
        motor.setControl(m_request.withVelocity(velo));
    }

    //-----------------------Smart Dashboard Controls-----------------------
    public void Enable_Smart_Dashboard_Controls() {
        SmartDashboard.putData(cfg.name + " Log Data", new InstantCommand(() -> Log_Data_To_Smart_Dashboard()));
        SmartDashboard.putData(cfg.name + " Enable Live Motion Magic", new InstantCommand(() -> Enable_Live_Motion_Magic()));
        SmartDashboard.putData(cfg.name + " Enable Live PID", new InstantCommand(() -> Enable_Live_PID(true)));
    }

    public void Log_Data_To_Smart_Dashboard() {
        if (!LogDataInit) {
            Update_Logged_Data();
            LogDataInit = true;
        }
        
    }

    private void Update_Logged_Data() {
        SmartDashboard.putNumber(cfg.name + " velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(cfg.name + " position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(cfg.name + " percent", motor.get());
    }

    public void Update() {
        if (LogDataInit) {
            Update_Logged_Data();
        }
    }

    //Test Motor
    public void Enable_Live_Testing(Command testCommand) {
        SmartDashboard.putData(testCommand);
    }

    public void Enable_Duty_Cycle_Testing() {
        SmartDashboard.putNumber(cfg.name + " Desired Percent", 0);
        SmartDashboard.putData(cfg.name + " Set desired persent", new InstantCommand(() -> Set_Live_Percent()));
        SmartDashboard.putData(cfg.name + " Test desired persent", new InstantCommand(() -> Duty_Cycle_Output(desired_Percent)));
    }

    public void Set_Live_Percent() {
        desired_Percent = SmartDashboard.getNumber(cfg.name + " Desired Percent", 0);
    }

    //Live PID
    public void Enable_Live_PID(boolean FOC) {
        if (!LivePIDInit) {
            SmartDashboard.putNumber(cfg.name + " kP", talonConfigs.Slot0.kP);
            SmartDashboard.putNumber(cfg.name + " kI", talonConfigs.Slot0.kI);
            SmartDashboard.putNumber(cfg.name + " kD", talonConfigs.Slot0.kD);
            SmartDashboard.putNumber(cfg.name + " setpoint", setPoint);

            SmartDashboard.putData(cfg.name + " Set PID", new InstantCommand(() -> Set_Live_PID()));

            if (!FOC) {
                SmartDashboard.putData("Test PID Position", new InstantCommand(() -> Test_Live_PID_Pos()));
                SmartDashboard.putData("Test PID Velocity", new InstantCommand(() -> Test_Live_PID_Velocity()));
            } else {
                SmartDashboard.putData("Test PID Position", new InstantCommand(() -> Test_Live_PID_Pos_FOC()));
                SmartDashboard.putData("Test PID Velocity", new InstantCommand(() -> Test_Live_PID_Velocity_FOC()));
            }
            LivePIDInit = true;
        }
    }

    public void Set_Live_PID() {
        // Set_PID_Slot_0(
        //     SmartDashboard.getNumber(cfg.name + " kP", 0),
        //     SmartDashboard.getNumber(cfg.name + " kI", 0),
        //     SmartDashboard.getNumber(cfg.name + " kD", 0)
        // );
        // setPoint = SmartDashboard.getNumber(cfg.name + " setpoint", 1);
    }

    private void Test_Live_PID_Pos() {
        PID_Position(setPoint);
    }

    private void Test_Live_PID_Pos_FOC() {
        PID_Position(setPoint, true);
    }

    private void Test_Live_PID_Velocity() {
        PID_Velocity(setPoint);
    }

    private void Test_Live_PID_Velocity_FOC() {
        PID_Velocity(setPoint, true);
    }

    //Live Motion Magic
    public void Enable_Live_Motion_Magic() {
        if (!LiveMotionMagicInit) {
            SmartDashboard.putNumber(cfg.name + " kP", talonConfigs.Slot0.kP);
            SmartDashboard.putNumber(cfg.name + " kI", talonConfigs.Slot0.kI);
            SmartDashboard.putNumber(cfg.name + " kD", talonConfigs.Slot0.kD);

            SmartDashboard.putNumber(cfg.name + " Crusie Velocity", talonConfigs.MotionMagic.MotionMagicCruiseVelocity);
            SmartDashboard.putNumber(cfg.name + " Acceleration", talonConfigs.MotionMagic.MotionMagicAcceleration);
            SmartDashboard.putNumber(cfg.name + " Jerk", talonConfigs.MotionMagic.MotionMagicJerk);

            SmartDashboard.putNumber(cfg.name + " setpoint", setPoint);

            SmartDashboard.putData(cfg.name + " Set PID", new InstantCommand(() -> Set_Live_PID()));
            SmartDashboard.putData(cfg.name + " Set Motion Magic Values", new InstantCommand(() -> Set_Live_Motion_Magic()));

            SmartDashboard.putData("Test Motion Magic Position", new InstantCommand(() -> Motion_Magic_Pos(setPoint, true)));
            SmartDashboard.putData("Test Motion Magic Velocity", new InstantCommand(() -> Motion_Magic_Velo(setPoint, true)));
            SmartDashboard.putData("Test PID Position", new InstantCommand(() -> Test_Live_PID_Pos_FOC()));
            SmartDashboard.putData("Test PID Velocity", new InstantCommand(() -> Test_Live_PID_Velocity_FOC()));
            LiveMotionMagicInit = true;
        }
        
    }

    public void Set_Live_Motion_Magic() {
        Motion_Magic_Setup(
            SmartDashboard.getNumber(cfg.name + " Crusie Velocity", 0),
            SmartDashboard.getNumber(cfg.name + " Acceleration", 0),
            SmartDashboard.getNumber(cfg.name + " Jerk", 0)
        );
        setPoint = SmartDashboard.getNumber(cfg.name + " setpoint", 1);
    }

    //-----------------------Simulation Support-----------------------
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void updateSimState(double deltaTime, double batteryVoltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateSimState'");
      }
    
    public void Enable_Sim() {
        if (Utils.isSimulation()) {
            startSimThread();
            motor.getSimState().setSupplyVoltage(12);
        }
        
    }

    public void Update_Sim() {
        var talonFXSim = motor.getSimState();
     
        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltage();
     
        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage);
        m_motorSimModel.update(0.020); // assume 20 ms loop time
     
        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratios)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPositionRotations());
        talonFXSim.setRotorVelocity(
           Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec())
        );
     }
    
}