// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.util.FuelSim;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(Constants.DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Turret turret = new Turret();
    public final Superstructure superstructure = new Superstructure(drivetrain, vision, intake, shooter, turret);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("shoot", Commands.runOnce(() -> superstructure.setRobotState(RobotState.SHOOTING)));
        NamedCommands.registerCommand("prepareThenShoot", Commands.runOnce(() -> superstructure.setRobotState(RobotState.PREP_SHOOT)).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> superstructure.setRobotState(RobotState.SHOOTING))));
        NamedCommands.registerCommand("idleShoot", Commands.runOnce(() -> superstructure.setRobotState(RobotState.IDLE_SHOOTING)));

        NamedCommands.registerCommand("intake", Commands.runOnce(() -> superstructure.setRobotState(RobotState.INTAKING)));
        NamedCommands.registerCommand("idleIntake", Commands.runOnce(() -> superstructure.setRobotState(RobotState.IDLE_INTAKING)));
        NamedCommands.registerCommand("deployIntake", Commands.runOnce(() -> intake.runDeploy(-5)));
        NamedCommands.registerCommand("retractIntake", Commands.runOnce(() -> intake.runDeploy(5)));
        NamedCommands.registerCommand("waitForIntakeToReach", new WaitUntilCommand(intake.hardStop));

        new EventTrigger("intake").onTrue(Commands.runOnce(() -> superstructure.setRobotState(RobotState.INTAKING)));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        configureFuelSim();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        driveCommand.addRequirements(drivetrain);
        drivetrain.setDefaultCommand(driveCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        Constants.OperatorConstants.driverController.leftBumper().whileTrue(drivetrain.faceDriveDirection);
        Constants.OperatorConstants.driverController.rightBumper().whileTrue(drivetrain.faceTarget);
        Constants.OperatorConstants.driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.OperatorConstants.driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        Constants.OperatorConstants.operatorController.leftTrigger().onTrue(Commands.runOnce(() -> superstructure.setRobotState(RobotState.INTAKING))).onFalse(Commands.runOnce(() -> superstructure.setRobotState(RobotState.IDLE_INTAKING)));
        Constants.OperatorConstants.operatorController.rightTrigger().whileTrue(NamedCommands.getCommand("prepareThenShoot")).onFalse(Commands.runOnce(() -> superstructure.setRobotState(RobotState.IDLE_SHOOTING)));

        Constants.OperatorConstants.operatorController.leftBumper().onTrue(Commands.runOnce(() -> intake.runDeploy(5))).onFalse(Commands.runOnce(() -> intake.runDeploy(0)));
        Constants.OperatorConstants.operatorController.rightBumper().onTrue(Commands.runOnce(() -> intake.runDeploy(-5))).onFalse(Commands.runOnce(() -> intake.runDeploy(0)));
        Constants.OperatorConstants.operatorController.y().onTrue(Commands.runOnce(() -> superstructure.setRobotState(RobotState.BACKUP))).onFalse(Commands.runOnce(() -> superstructure.setRobotState(RobotState.IDLE)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command driveCommand = Commands.run(() -> {
            drivetrain.setControl(
                drive.withVelocityX(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftY(), 0.1) * Constants.DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getLeftX(), 0.1) * Constants.DriveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(Constants.OperatorConstants.driverController.getRightX(), 0.1) * Constants.DriveConstants.MaxAngularRate)); // Drive counterclockwise with negative X (left));
        }
    );

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.spawnStartingFuel();
        instance.registerRobot(
            Meter.convertFrom(28, Inches), 
            Meter.convertFrom(28, Inches), 
            Meter.convertFrom(5.5, Inches), 
            () -> drivetrain.getState().Pose, 
            () -> drivetrain.getState().Speeds);

        instance.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
    }
}
