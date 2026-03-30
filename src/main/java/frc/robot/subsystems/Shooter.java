package frc.robot.subsystems;

import javax.xml.transform.sax.SAXTransformerFactory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.khoodMotorId);
    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kshooterMotorId);
    private final TalonFX indexerMotor = new TalonFX(ShooterConstants.kindexerMotorId);
    private final PIDController hoodPID = new PIDController(0.05, 0,0);
    private double targetAngle = 0;
    private boolean isHoming = false;

    public Shooter() {
        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        hoodMotor.getConfigurator().apply(hoodMotorConfig);

        TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        shooterMotor.getConfigurator().apply(shooterMotorConfig);

        TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
        indexerMotor.getConfigurator().apply(indexerMotorConfig);

    }

    // for adjusting rangeMotor Angle

    public double getHoodMotorAngleDegrees() {
        double motorRotations = hoodMotor.getPosition().getValueAsDouble();
        double angleDegrees = motorRotations * ShooterConstants.gearRatio * 360;
        return angleDegrees;
    }
    // public void setToRightAngle(double goal) {
    //     double safeGoal = MathUtil.clamp(goal, 0.0, 60.0);
    //     hoodMotor.set(hoodPID.calculate(getHoodMotorAngleDegrees(), safeGoal)); 
    // }

    // 讓外部設定目標的方法
    public void setTargetAngle(double degrees) {
        targetAngle = MathUtil.clamp(degrees, -1, ShooterConstants.hoodBiggestAngle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean atSetAngle() {
        return hoodPID.atSetpoint();
    }

    public void stopHoodMotor() {
        hoodMotor.stopMotor();
    }

    public void resetShooterAngle() {
        hoodMotor.setPosition(0);
        this.targetAngle = 0;
    }

    public boolean isStalled() {
        return hoodMotor.getStatorCurrent().getValueAsDouble() >= 20.0 ;
    }  

    public void controllerMoveAngle(double volt) {
        hoodMotor.set(volt);
    }

    public double getHoodVoltage() {
        return hoodMotor.getMotorVoltage().getValueAsDouble();
    }


    // for shooterMotor
    public double getShooterVoltage() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public void shooterSetVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }

    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    // for indexerMotor
    public void runIndexerMotor(double speed) {
        indexerMotor.setVoltage(speed);
    }
    
    public void stopIndexerMotor() {
        indexerMotor.stopMotor();
    }


    // Auto
    public void autoAim (double distance) {
        if(distance < 0.5){
            targetAngle = ShooterConstants.closestAngle;
        }
        targetAngle = ShooterConstants.table.get(distance);
        this.setTargetAngle(targetAngle);
    }
 
    public boolean isReadyToShoot() {
        boolean angleReady = Math.abs(getHoodMotorAngleDegrees() - targetAngle) < 1;
        return angleReady;
    }

    //
    public void setHomingMode(boolean homing) {
        this.isHoming = homing;
    }

    public void driveManual(double vol) {
        hoodMotor.setVoltage(vol);
    }





    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Hood/Target", targetAngle);
        SmartDashboard.putNumber("Shooter/Hood/Actual Angle", getHoodMotorAngleDegrees());
        SmartDashboard.putNumber("Shooter/Hood/HoodMotorVoltage", getHoodVoltage());
        SmartDashboard.putNumber("Shooter Voltage", getShooterVoltage());
        SmartDashboard.putData("Shooter hoodMotor PID", hoodPID);
        SmartDashboard.putBoolean("isHoming", isHoming);


        if (!isHoming) {
        double hoodPidOutput = hoodPID.calculate(getHoodMotorAngleDegrees(), getTargetAngle());
        double feedforward = ShooterConstants.kG * Math.cos(Math.toRadians(getHoodMotorAngleDegrees()));
        double totalVoltage = hoodPidOutput + feedforward;
        double safeVoltage = MathUtil.clamp(totalVoltage, -12.0, 12.0);
        hoodMotor.setVoltage(safeVoltage);
        }
    }
 
}
