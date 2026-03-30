package frc.robot.commands.shooter;

import javax.sound.midi.ControllerEventListener;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterMove extends Command{
    private final Shooter shooter;
    private final CommandXboxController controller;
    double shooterSpeed = 0;
    double shooterAngle = 0;
    double indexerSpeed = 0;

    public ShooterMove(Shooter shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(controller.getHID().getAButton()) {
            shooterSpeed = 11;
            indexerSpeed = -8;
        } else {
            shooterSpeed = 0;
            indexerSpeed = 0;
        }
        shooter.shooterSetVoltage(shooterSpeed);
        shooter.runIndexerMotor(indexerSpeed);

        // 角度控制：按一下增加，不按時 m_targetAngle 會被子系統記住，不會歸零
        if(controller.getHID().getBButtonPressed()) {
            double currentGoal = shooter.getTargetAngle();
            currentGoal += 1;
            shooter.setTargetAngle(currentGoal);
        }else if(controller.getHID().getYButton()) {
            shooter.setTargetAngle(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopHoodMotor();
        shooter.stopShooterMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    
}
