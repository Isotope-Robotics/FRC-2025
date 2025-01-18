package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
    public static SparkMax conveyorMotor;
    public static DigitalInput conveyorSensor;

    private static Conveyor m_Instance = null;

    public Conveyor(int conveyorMotorID, int conveyorSensorID) {
        conveyorMotor = new SparkMax(conveyorMotorID, MotorType.kBrushless);
        conveyorSensor = new DigitalInput(conveyorSensorID);
    }
    //sets the speed of the motor connected to the conveyor belt from -1 to 1 based of parameter
    public void setSpeed(double speed) {
        conveyorMotor.set(speed);
    }
    //makes the motor connected to the conveyor belt stop moving
    public void turnOffConveyor() {
        conveyorMotor.set(0);
    }
    //returns if there is something(coral) in the of the sensor before the conveyor belt
    public boolean isConveyorClear(){
        return conveyorSensor.get();
    }

    //defines motor and sensor for class
    public static Conveyor getInstance() {
        if (m_Instance == null)
            m_Instance = new Conveyor(Constants.Conveyor.conveyorMotorID, Constants.Conveyor.conveyorSensorID);
        return m_Instance;
    }
}
