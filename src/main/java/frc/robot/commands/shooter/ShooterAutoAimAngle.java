package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.Constants.ShooterConstants;;

public class ShooterAutoAimAngle extends Command{
    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;
    
    public ShooterAutoAimAngle(Shooter shooter, CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double distance = drivetrain.getRobotToHubDistance();
        shooter.autoAim(distance);
        if(shooter.isReadyToShoot()){
            shooter.shootshoot(true);
        }else if(controller.getHID().getLeftBumperButton()){
            shooter.shootshoot(true);
        }else{
            shooter.shootshoot(false);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopHoodMotor();
        shooter.shootshoot(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
