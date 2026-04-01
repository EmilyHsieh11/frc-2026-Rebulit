package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterAuto extends Command{

    private final Shooter shooter;
    private final double targetAngle;

    public ShooterAuto(Shooter shooter, double targetAngle){
        this.shooter = shooter;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooter.setTargetAngle(targetAngle);
        if(Math.abs(shooter.getTargetAngle() - targetAngle) <= 1){
            shooter.shootshoot(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopHoodMotor();
        shooter.shootshoot(false);
    }
    
}
