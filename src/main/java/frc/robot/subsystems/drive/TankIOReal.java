package frc.robot.subsystems.drive;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class TankIOReal implements TankIO {

    private final TalonFX leftLead = new TalonFX(1);
    private final TalonFX leftFollow1 = new TalonFX(2); 
    private final TalonFX leftFollow2 = new TalonFX(3); 

    private final TalonFX rightLead = new TalonFX(4);
    private final TalonFX rightFollow1 = new TalonFX(5); 
    private final TalonFX rightFollow2 = new TalonFX(6);
    
    private final DifferentialDrive drive = new DifferentialDrive(leftLead, rightLead);

    public TankIOReal() {
        boolean invertLeft = true;
        boolean invertRight = false;
        this.leftLead.setInverted(invertLeft);
        this.rightLead.setInverted(invertRight);

        //configure Followers
        leftFollow1.setControl(new Follower(leftLead.getDeviceID(), false));
        leftFollow2.setControl(new Follower(leftLead.getDeviceID(), false));
        rightFollow1.setControl(new Follower(rightLead.getDeviceID(), false));
        rightFollow2.setControl(new Follower(rightLead.getDeviceID(), false));

    }

    public void updateInputs(TankIOInputs inputs) {}

    public void drive(double move, double rotate) {
        // We invert the axis because - is forward on the joystick, 
        // but we want + output as forward for the motors
        drive.arcadeDrive(-move, -rotate);
    }

    public void stop() {
        drive.arcadeDrive(0.0, 0.0);
    }
    
}
