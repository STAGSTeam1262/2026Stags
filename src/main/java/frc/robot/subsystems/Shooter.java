package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    Superstructure superstructure;

    TalonFX feederMotor = new TalonFX(Constants.CANConstants.FeederID, Constants.CANConstants.canivore);
    DoublePublisher feederSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Speed").publish();
    DoublePublisher feederVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Feeder/Voltage").publish();

    public Shooter() {

    }

    public void stop() {
        feederMotor.stopMotor();
    }

    public void runFeeder(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        feederSpeedPublisher.set(feederMotor.getVelocity().getValueAsDouble());
        feederVoltagePublisher.set(feederMotor.getMotorVoltage().getValueAsDouble());
    }
}
