package frc.robot.common;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;


public class ConfigTalonFX {

    Config cfg;

    //Creates Motor
    public TalonFX motor;

    TalonFXConfiguration talonConfigs;

    public ConfigTalonFX (ConfigTalonFX.Config cfg) {
        this.cfg = cfg;

        motor = new TalonFX(this.cfg.canID);
        talonConfigs = new TalonFXConfiguration();
        applyConfigs();
    }

    public static final class CurrentLimitConfigs extends LoadableConfig {
        public double intakelimit;
        public double shootlimit;
        public double climblimit;
        public double defenselimit;
        public double cyclelimit;

        public CurrentLimitConfigs(){};
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

        public double kCruiseVelocity;
        public double kAcceleration;
        public double kMaxUnitsLimit;
        public double kMinUnitsLimit;
        public double kEnableSupplyCurrentLimit;
        public double kSupplyCurrentLimit;
        public double kSupplyCurrentThreshold;
        public double kSupplyCurrentTimeout;
        public double kMaxForwardOutput;
        public double kMaxReverseOutput;
    
        public Slot(){};
    }

    public static final class Config extends LoadableConfig {
        public String name;
        public int canID;
        public boolean inverted;
        public boolean isBreakMode;

		public Config(String filename) {
			super.load(this, filename);
			LoadableConfig.print(this);
		}
	}

    public static final class PIDConfig extends LoadableConfig {
        public String name;
        public int canID;
        public boolean inverted;

        public BasicSlot slot0;
        public BasicSlot slot1;
        public BasicSlot slot2;

		public PIDConfig(String filename) {
			super.load(this, filename);
			LoadableConfig.print(this);
		}
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

    public void applyConfigs() {

        motor.getConfigurator().apply(talonConfigs);
    }
}
