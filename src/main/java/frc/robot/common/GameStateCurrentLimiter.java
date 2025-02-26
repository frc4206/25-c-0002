package frc.robot.common;
import java.util.ArrayList;
import java.util.HashMap;
import org.team4206.battleaid.common.LoadableConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;

public class GameStateCurrentLimiter {
    
    State currentState;
    HashMap<String, TalonFX[]> SubsystemList = new HashMap<>();

    HashMap<String, Double[]> IntakeCurrentLimits = new HashMap<>();
    HashMap<String, Double[]> ShootCurrentLimits = new HashMap<>();
    HashMap<String, Double[]> ClimbCurrentLimits = new HashMap<>();
    HashMap<String, Double[]> DefenseCurrentLimits = new HashMap<>();
    HashMap<String, Double[]> CycleCurrentLimits = new HashMap<>();

    public ArrayList<HashMap<String, Double[]>> CurrentLimits = new ArrayList<>();

    public enum State {
        INTAKE,
        SHOOT,
        CLIMB,
        DEFENSE,
        CYCLE
    }

    public class  Config  extends LoadableConfig {

        public Config(String filename){
            super.load(this, filename);
            LoadableConfig.print(this);
        }
    }

    public GameStateCurrentLimiter() {
        CurrentLimits.add(IntakeCurrentLimits);
        CurrentLimits.add(ShootCurrentLimits);
        CurrentLimits.add(ClimbCurrentLimits);
        CurrentLimits.add(DefenseCurrentLimits);
        CurrentLimits.add(CycleCurrentLimits);
    }

    public void AddSubsystem(String subName, TalonFX[] motorList) {
        SubsystemList.put(subName, motorList);
        for (HashMap<String, Double[]> i : CurrentLimits) {
            i.put(subName, new Double[motorList.length]);
        }
    }

    public void SetState(State newState) {
        currentState = newState;
        switch (currentState) {
            case INTAKE:
                SetCurrentLmits(IntakeCurrentLimits);
                break;
            case SHOOT:
                SetCurrentLmits(ShootCurrentLimits);
                break;
            case CLIMB:
                SetCurrentLmits(ClimbCurrentLimits);
                break;
            case DEFENSE:
                SetCurrentLmits(DefenseCurrentLimits);
                break;
            case CYCLE:
                SetCurrentLmits(CycleCurrentLimits);
                break;
            default:
                break;
        }
    }

    public void SetCurrentLmits(HashMap<String, Double[]> SubsystemCurrentLimits) {
        for (String i : SubsystemList.keySet()) {
            for(int j = 0; j < SubsystemList.get(i).length; j++) {
                TalonFXConfiguration currentlimit = new TalonFXConfiguration()
                    .withCurrentLimits(
                        new CurrentLimitsConfigs()
                            .withSupplyCurrentLimit(SubsystemCurrentLimits.get(i)[j])
                    );
                SubsystemList.get(i)[j].getConfigurator().apply(currentlimit);
            }
        }
    }

    public void PopulateSubsystemLimits(String subName, ArrayList<Double[]> limits) {
        for(int i = 0; i < CurrentLimits.size(); i++) {
            CurrentLimits.get(i).put(subName, limits.get(i));
        }

    }
}