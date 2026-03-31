package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase {

    Drivetrain drivetrain;
    Vision vision;
    Intake intake;
    Turret turret;
    Shooter shooter;
    public Pose2d allianceHubPosition = Constants.FieldConstants.blueHubCenter;
    public ArrayList<Pose2d> bluePassingPositions = new ArrayList<>();
    public ArrayList<Pose2d> redPassingPositions = new ArrayList<>();
    Alliance currentAlliance = Alliance.Blue;
    public ShotData shotData;

    DoublePublisher shotDistancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("ShotData/Shot Distance").publish();
    DoublePublisher shotAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("ShotData/Shot Angle").publish();
    StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("ShotData/Current Target", Pose2d.struct).publish();

    public RobotState robotState = RobotState.IDLE;
    StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/Robot State").publish();

    public FallbackMode fallbackMode = FallbackMode.ALL_OPERATIONAL; // Please stay this way. Thanks.
    StringPublisher fallbackPublisher = NetworkTableInstance.getDefault().getStringTopic("Subsystems/Superstructure/Fallback Mode").publish();

    public CurrentAllianceShift shift = CurrentAllianceShift.BOTHACTIVE;
    StringPublisher shiftPublisher = NetworkTableInstance.getDefault().getStringTopic("Match/Current Alliance Shift").publish();
    public Trigger visionIsFine;
    BooleanPublisher visionWorksPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Subsystems/Vision/Is OK").publish();

    public Superstructure(Drivetrain drivetrain, Vision vision, Intake intake, Shooter shooter, Turret turret) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;

        bluePassingPositions.add(Constants.FieldConstants.blueLeft);
        bluePassingPositions.add(Constants.FieldConstants.blueRight);
        redPassingPositions.add(Constants.FieldConstants.redLeft);
        redPassingPositions.add(Constants.FieldConstants.redRight);

        drivetrain.provideSubsystemAccessToSuperstructure(this);
        vision.provideSubsystemAccessToSuperstructure(this);
        intake.provideSubsystemAccessToSuperstructure(this);
        shooter.provideSubsystemAccessToSuperstructure(this);
        turret.provideSubsystemAccessToSuperstructure(this);

        visionIsFine = vision.tag1Connected.or(vision.tag2Connected).or(vision.tag3Connected).or(() -> RobotBase.isSimulation());
    }

    public record ShotData(Translation2d robotPosition, Translation2d shotPosition, double shotDistance, Rotation2d shotAngle) {}

    public ShotData calculateShotData(Translation2d robotPosition, Translation2d target) {
        return new ShotData(robotPosition, target, getDistanceToTarget(target), getAngleToTarget(target));
    }

    public Translation2d getTarget() {
        Pose2d target = allianceHubPosition;
        if (canPass()) {
            if (currentAlliance == Alliance.Blue) {
                target = getRobotPose().nearest(bluePassingPositions);
            } else {
                target = getRobotPose().nearest(redPassingPositions);
            }
        }
        return target.getTranslation();
    }

    public boolean canPass() {
        boolean canPass = false;
        if (currentAlliance == Alliance.Blue) {
            if (getRobotPose().getX() > 5) {
                canPass = true;
            }
        } else {
            if (getRobotPose().getX() < 12) {
                canPass = true;
            }
        }
        return canPass;
    }

    public double getDistanceToTarget(Translation2d target) {
        double distance = drivetrain.getState().Pose.getTranslation().getDistance(target);
        SmartDashboard.putNumber("Distance", distance);
        return distance;
    }

    public Rotation2d getAngleToTarget(Translation2d target) {
        Pose2d drivePose = drivetrain.getState().Pose;
        
        double deltaX = target.getX() - drivePose.getX();
        double deltaY = target.getY() - drivePose.getY();
        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return Rotation2d.fromDegrees(angle);
    }
    
    public enum RobotState {
        CLIMBER_DOWN,
        CLIMBER_UP,
        IDLE,
        IDLE_INTAKING,
        IDLE_SHOOTING,
        INTAKING,
        SHOOTING,
        SHOOTING_INTAKING,
        PREP_SHOOT,
        PREP_SHOOT_INTAKING,
        BACKUP,
        DEBUG_SHOOT;
    }

    public enum FallbackMode {
        ALL_OPERATIONAL,
        VISION_DOWN,
        TURRET_DOWN,
        ALL_DOWN;
    }

    public RobotState shooterToggle() {
        if (robotState == RobotState.SHOOTING || robotState == RobotState.SHOOTING_INTAKING) {
            return RobotState.IDLE_SHOOTING;
        } else {
            return RobotState.SHOOTING;
        }
    }

    public void setRobotState(RobotState state) {
        if ((state == RobotState.INTAKING && robotState == RobotState.SHOOTING) || (state == RobotState.SHOOTING && robotState == RobotState.INTAKING) || (state == RobotState.SHOOTING && robotState == RobotState.PREP_SHOOT_INTAKING)) {
            state = RobotState.SHOOTING_INTAKING;
        } else if ((state == RobotState.INTAKING && robotState == RobotState.PREP_SHOOT) || (state == RobotState.PREP_SHOOT && robotState == RobotState.INTAKING)) {
            state = RobotState.PREP_SHOOT_INTAKING;
        } else if (state == RobotState.IDLE_INTAKING) {
            if (robotState == RobotState.INTAKING) {
                state = RobotState.IDLE;
            } else if (robotState == RobotState.SHOOTING_INTAKING) {
                state = RobotState.SHOOTING;
            } else if (robotState == RobotState.PREP_SHOOT_INTAKING) {
                state = RobotState.PREP_SHOOT;
            } else {
                state = RobotState.IDLE;
            }
        } else if (state == RobotState.IDLE_SHOOTING) {
            if (robotState == RobotState.SHOOTING) {
                state = RobotState.IDLE;
            } else if (robotState == RobotState.SHOOTING_INTAKING) {
                state = RobotState.INTAKING;
            } else if (robotState == RobotState.PREP_SHOOT) {
                state = RobotState.IDLE;
            } else if (robotState == RobotState.PREP_SHOOT_INTAKING) {
                state = RobotState.INTAKING;
            } else {
                state = RobotState.IDLE;
            }
        }
        this.robotState = state;
        applyRobotState();
    }

    public void applyRobotState() {
        if (robotState == RobotState.IDLE) {
            idleIntake();
            idleShoot();
        } else if (robotState == RobotState.DEBUG_SHOOT) {
            debugShoot();
        } else if (robotState == RobotState.INTAKING) {
            intake();
            idleShoot();
        } else if (robotState == RobotState.SHOOTING) {
            shoot();
            idleIntake();
        } else if (robotState == RobotState.SHOOTING_INTAKING) {
            intake();
            shoot();
        } else if (robotState == RobotState.CLIMBER_UP) {
            climberUp();
        } else if (robotState == RobotState.CLIMBER_DOWN) {
            climberDown();
        } else if (robotState == RobotState.PREP_SHOOT) {
            prepShoot();
            idleIntake();
        } else if (robotState == RobotState.PREP_SHOOT_INTAKING) {
            prepShoot();
            intake();
        } else if (robotState == RobotState.BACKUP) {
            backup();
            idleIntake();
        }
    }

    public void prepShoot() {
        shooter.setState(Shooter.State.PREP_SHOT);
    }

    public void backup() {
        shooter.setState(Shooter.State.BACKUP);
    }

    public void debugShoot() {
        shooter.setState(Shooter.State.DEBUG);
    }

    public void climberDown() {
        // Move climber down.
    }

    public void climberUp() {
        // Move climber up.
    }

    public void idleIntake() {
        intake.runRoller(0);
    }

    public void intake() {
        intake.runRoller(6);
    }

    public void idleShoot() {
        shooter.setState(Shooter.State.IDLE);
    }

    public void shoot() {
        shooter.setState(Shooter.State.SHOOTING);
    }

    public void stop() {
        shooter.stop();
        intake.stop();
        turret.stop();
    }

    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRobotPose().getRotation());
    }

    @Override
    public void periodic() {
        statePublisher.set(robotState.toString());
        fallbackPublisher.set(fallbackMode.toString());
        visionWorksPublisher.set(visionIsFine.getAsBoolean());

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

        if (DriverStation.getAlliance().isPresent()) {
            currentAlliance = DriverStation.getAlliance().get();
            allianceHubPosition = currentAlliance == Alliance.Blue ? Constants.FieldConstants.blueHubCenter : Constants.FieldConstants.redHubCenter;
        }

        shotData = calculateShotData(getRobotPose().getTranslation(), getTarget());
        
        shotAnglePublisher.set(shotData.shotAngle().plus(Rotation2d.k180deg).getDegrees());
        shotDistancePublisher.set(shotData.shotDistance());
        targetPosePublisher.set(new Pose2d(shotData.shotPosition, Rotation2d.kZero));

        shiftPublisher.set(shift.color.toString());
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

}
