package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase {

    Drivetrain drivetrain;
    public Pose2d allianceHubPosition = Constants.FieldConstants.blueHubCenter;

    public Superstructure(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    public enum RobotState {
        DISABLED,
        GROUND_INTAKING,
        STATION_INTAKING,
        SHOOTING,
        PASSING;

        RobotState() {
            
        }

    }

    public enum CurrentAllianceShift {
        BLUE("#0000FF"),
        RED("#FF0000"),
        BOTHACTIVE("#FFFFFF");

        String color;

        CurrentAllianceShift(String color) {
            this.color = color;
        }
    }

    public RobotState robotState = RobotState.DISABLED;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/Robot State").publish();

    public CurrentAllianceShift shift = CurrentAllianceShift.BOTHACTIVE;
    StringPublisher shiftPublisher = NetworkTableInstance.getDefault().getStringTopic("Match/Current Alliance Shift").publish();

    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void periodic() {
        statePublisher.set(robotState.toString());

        String autoWinner = DriverStation.getGameSpecificMessage();
        double time = DriverStation.getMatchTime();
        if (DriverStation.isTeleopEnabled()) {
            if (autoWinner.length() > 0) {
                switch (autoWinner.charAt(0)) {
                    case 'B':
                        if (time < 30 || time > 130) {
                            shift = CurrentAllianceShift.BOTHACTIVE;
                        } else if ((time >= 30 && time < 55) || (time >= 80 && time < 105)) {
                            shift = CurrentAllianceShift.BLUE;
                        } else {
                            shift = CurrentAllianceShift.RED;
                        }
                        break;
                    case 'R':
                        if (time < 30 || time > 130) {
                            shift = CurrentAllianceShift.BOTHACTIVE;
                        } else if ((time >= 30 && time < 55) || (time >= 80 && time < 105)) {
                            shift = CurrentAllianceShift.RED;
                        } else {
                            shift = CurrentAllianceShift.BLUE;
                        }
                        break;
                    default:
                        shift = CurrentAllianceShift.BOTHACTIVE;
                }
            }
        }

        shiftPublisher.set(shift.color.toString());
    }

}
