package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOReal implements ShooterIO{

    private final TalonFX shooterLeft = new TalonFX(21); 
    private final TalonFX shooterRight = new TalonFX(22);

    private final TalonFX feedLeft = new TalonFX(23);
    private final TalonFX feedRight = new TalonFX(24);


    public ShooterIOReal(){

        shooterLeft.setInverted(true);
        feedLeft.setInverted(true);
        shooterRight.setInverted(false);
        feedRight.setInverted(false);
        
    }

    public void updateInputs(ShooterIOInputs inputs) {}

    public void setVoltages(double leftShooterVolts, double rightShooterVolts,
                            double feedVolts) {
                                        shooterLeft.setVoltage(leftShooterVolts);
                                        shooterRight.setVoltage(rightShooterVolts);
                                        feedLeft.setVoltage(feedVolts);
                                        feedRight.setVoltage(feedVolts);

                                    updateRPM();
                                    }
    
    public void setLeftShooterVelocity(double velocityRotPerMin, double ffVolts) {
        shooterLeft.setControl(
        new VelocityVoltage(
            velocityRotPerMin/60, 0.0, true, ffVolts, 0, false, false, false));
        
        updateRPM();
        }

    public void setRightShooterVelocity(double velocityRotPerMin, double ffVolts) {
        shooterRight.setControl(
        new VelocityVoltage(
            velocityRotPerMin/60, 0.0, true, ffVolts, 0, false, false, false));
        updateRPM();
        }

    public void setLeftFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    public void setRightFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    @Override
    public void setFeedVoltage(double ffVolts) {
        
        feedLeft.setVoltage(ffVolts);
        feedRight.setVoltage(ffVolts);
        updateRPM();
    }

    public void setVelocity(double velocityRadPerSec, double ffVolts) {}

    public void stop() {}

    public void configurePID(double kP, double kI, double kD /*double kV, double kS*/) { 
        var shooterConfig = new Slot0Configs();
        shooterConfig.kP = kP;
        shooterConfig.kI = kI;
        shooterConfig.kD = kD;
        shooterConfig.kV = 0.0; //kV;
        shooterConfig.kS = 0.0; //kS;
        
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
       // feedLeft.getConfigurator().apply(shooterConfig);
         // feedRight.getConfigurator().apply(shooterConfig);
    }

    public void updateRPM(){
    SmartDashboard.putNumber("leftShooter rpm",shooterLeft.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("rightShooter rpm",shooterRight.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("feed rpm",feedLeft.getVelocity().getValueAsDouble());
    }


    
}
