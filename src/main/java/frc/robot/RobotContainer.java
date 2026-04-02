// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.intake.IntakeMove;
import frc.robot.commands.shooter.ShooterAuto;
import frc.robot.commands.shooter.ShooterAutoAimAngle;
import frc.robot.commands.shooter.ShooterMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xBox = new CommandXboxController(0);
    private final CommandPS5Controller ps5 = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake intake = new Intake();
    private final IntakeMove intakeLowerMove = new IntakeMove(intake,IntakeConstants.kpivotLowered,IntakeConstants.kIntakeVolts);
    private final IntakeMove intakeHangUp = new IntakeMove(intake, IntakeConstants.kpivotRaised, 0.0);

    private final Shooter shooter = new Shooter();
    private final ShooterMove shooterMove = new ShooterMove(shooter, xBox);
    private final ShooterAuto shooterAuto = new ShooterAuto(shooter, ShooterConstants.hoodBiggestAngle);
    private final ShooterAutoAimAngle shooterAimAngle = new ShooterAutoAimAngle(shooter, drivetrain, xBox);




    // /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final PhotonVision vision = new PhotonVision(() -> drivetrain.getState().Pose);

    public RobotContainer() {
        NamedCommands.registerCommand("Left path shoot", shooterAuto);
        NamedCommands.registerCommand("Intaking", intakeLowerMove);
        NamedCommands.registerCommand("TimedShoot", new ParallelDeadlineGroup(new WaitCommand(2.0), shooterAuto));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // shooter.setDefaultCommand(shooterMove);
        
        xBox.rightBumper().onTrue(intakeHangUp);
        xBox.leftBumper().onTrue(intakeLowerMove);

        drivetrain.setVisionSubsystem(vision);

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xBox.getLeftY() * MaxSpeed  ) // Drive forward with negative Y (forward)
                    .withVelocityY(-xBox.getLeftX() * MaxSpeed ) // Drive left with negative X (left)
                    .withRotationalRate(-xBox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // ps5.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // // Idle while the robot is disabled. This ensures the configured
        // // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        // joystick.x().whileTrue(getAutoAimDriveCommand().alongWith(shooterAimAngle));
        // joystick.y().whileTrue(shooterAimAngle);

        xBox.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // xBox.back().onTrue(drivetrain.runOnce(drivetrain::));

    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            shooterHomingCommand(),
            intakeHomingCommand(),
            new ParallelDeadlineGroup(
                autoChooser.getSelected(), 
                new IntakeMove(intake, IntakeConstants.kpivotLowered, IntakeConstants.kIntakeVolts))
        );
    
    }

    Command getAutoAimDriveCommand() {
        return drivetrain.applyRequest(() -> new SwerveRequest.FieldCentricFacingAngle()
        .withVelocityX(-xBox.getLeftY() * MaxSpeed)
        .withVelocityY(-xBox.getLeftX() * MaxSpeed)
        .withTargetDirection(drivetrain.getRobotHubHeading())
        );  
    }


    public Command shooterHomingCommand() {
        return Commands.sequence(
            // 1. 開始歸零：打開開關，避開 periodic 的 PID
            Commands.runOnce(() -> shooter.setHomingMode(true)),

            Commands.run(() -> shooter.driveManual(-0.5), shooter) 
                    .until(() -> shooter.isStalled())
                    .withTimeout(2.0),
            
            // 3. 結束歸零：重設位置，關閉開關
            Commands.runOnce(() -> {
                shooter.stopHoodMotor();
                shooter.resetShooterAngle(); // 設定編碼器為 0
                shooter.setTargetAngle(0);   // 設定目標為 0
                shooter.setHomingMode(false); // 把控制權還給 periodic 的 PID
            }, shooter)
        );
    }

    public Command intakeHomingCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.resetPivotEncoder(), intake)
        );
    }


    
}
