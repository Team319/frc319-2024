package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOProto implements ShooterIO{

    private final TalonFX shooterLeft = new TalonFX(21); 
    private final TalonFX shooterRight = new TalonFX(22);

    private final TalonFX feedLeft = new TalonFX(23);
    private final TalonFX feedRight = new TalonFX(24);

    public ShooterIOProto(){

        shooterLeft.setInverted(true);
        feedLeft.setInverted(true);
        shooterRight.setInverted(false);
        feedRight.setInverted(false);
        
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {}

    @Override
    public void setFeedVoltage(double ffVolts) {
        feedLeft.setVoltage(ffVolts);
        feedRight.setVoltage(ffVolts);
        updateRPM();
    }

    @Override
    public void setVoltages(double leftShooterVolts, double rightShooterVolts,double feedVolts) {
        shooterLeft.setVoltage(leftShooterVolts);
        shooterRight.setVoltage(rightShooterVolts);
        setFeedVoltage(feedVolts);
        updateRPM();
    }

    public void setRightFeedVoltage(double velocityRadPerSec, double ffVolts) {
        feedRight.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }

    public void setLeftFeedVoltage(double velocityRadPerSec, double ffVolts) {
        feedLeft.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }

    @Override
    public void setLeftShooterVelocity(double velocityRadPerSec, double ffVolts) {
        shooterLeft.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }

    @Override
    public void setRightShooterVelocity(double velocityRadPerSec, double ffVolts) {
        shooterRight.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
    }


    @Override
    public void setShooterVelocity(double velocityRadPerSec, double ffVolts) {
        setLeftShooterVelocity(velocityRadPerSec, ffVolts);
        setRightShooterVelocity(velocityRadPerSec, ffVolts);
        updateRPM();
    }

    @Override
    public void setFeedVelocity(double velocityRadPerSec, double ffVolts) {
        setLeftFeedVoltage(velocityRadPerSec, ffVolts);
        setRightFeedVoltage(velocityRadPerSec, ffVolts);
        updateRPM();
    }

    @Override
    public void stop() {
        shooterLeft.stopMotor();
        shooterRight.stopMotor();
        feedLeft.stopMotor();
        feedRight.stopMotor();  
    }

    @Override
    public void configurePID(double kP, double kI, double kD ) { 
        var shooterConfig = new Slot0Configs();
        shooterConfig.kP = kP;
        shooterConfig.kI = kI;
        shooterConfig.kD = kD;
        
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
        feedLeft.getConfigurator().apply(shooterConfig);
        feedRight.getConfigurator().apply(shooterConfig);
    }

    public void updateRPM(){
    SmartDashboard.putNumber("leftShooter rpm",shooterLeft.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("rightShooter rpm",shooterRight.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("feed rpm",feedLeft.getVelocity().getValueAsDouble());
    }


    
}
