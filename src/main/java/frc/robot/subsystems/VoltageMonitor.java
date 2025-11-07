package frc.robot.subsystems;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

public class VoltageMonitor {


    private static final VoltageMonitor instance = new VoltageMonitor();
    

    public VoltageMonitor() {
        
    }

    public static VoltageMonitor getInstance() {
        return instance;
    }

    public static void printVoltage() {
        System.out.println("Current voltage is: " + RobotController.getBatteryVoltage());
    }
    public static void printDraw() {
        PowerDistribution pdh = new PowerDistribution();
        System.out.println(" Current draw is: " + pdh.getTotalCurrent());
    }
}
