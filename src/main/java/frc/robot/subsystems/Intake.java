package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    Superstructure superstructure;

    public Intake() {
        configureMotors();
    }

    public void configureMotors() {

    }

    public void stop() {

    }

    public void provideSubsystemAccessToSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        
    }

}