package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;


public class chassis extends SubsystemBase {

    private final CANSparkMax FL = new CANSparkMax(Constants.FL, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax FR = new CANSparkMax(Constants.FR, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax RL = new CANSparkMax(Constants.RL, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax RR = new CANSparkMax(Constants.RR, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(FL, RL);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(FR, RR);
    public DifferentialDrive tank = new DifferentialDrive(leftMotors, rightMotors);


    public chassis(){
        FL.restoreFactoryDefaults();
        FR.restoreFactoryDefaults();
        RL.restoreFactoryDefaults();
        RR.restoreFactoryDefaults();

        FL.setInverted(true);
        FR.setInverted(true);

        RL.follow(FL);
        RR.follow(FR);

        FL.setIdleMode(IdleMode.kBrake);
        FR.setIdleMode(IdleMode.kBrake);
        RL.setIdleMode(IdleMode.kBrake);
        RR.setIdleMode(IdleMode.kBrake);
    }
    public void drive(double X, double Y){
        tank.arcadeDrive(-Y,-X);
    }


    
}
