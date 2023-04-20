package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveJoystickComplexCmd extends CommandBase{

    private final Supplier<Double> drivePower;
    private final Supplier<Double> turnY;
    private final Supplier<Double> turnX;


    private final SwerveSubsystemKrish swerve;

    public SwerveJoystickComplexCmd(Supplier<Double> drivePower, Supplier<Double> turnY, Supplier<Double> turnX, SwerveSubsystemKrish swerve){
        this.drivePower = drivePower;
        this.turnY = turnY;
        this.turnX = turnX;

        this.swerve = swerve;

        addRequirements(swerve);
    }



    @Override
    public void initialize(){
       

    }

    @Override
    public void execute(){
        double realTimeDrivePower =  -1 * drivePower.get() / 3;
        double realTimeTurnPowerY =  1 * turnY.get() / 3;
        double realTimeTurnPowerX =  1 * turnX.get() / 3;


        if(realTimeDrivePower > 0.05 || realTimeTurnPowerX > 0.05 || realTimeTurnPowerY > 0.05){

            double angle = Math.atan2(realTimeTurnPowerY, realTimeTurnPowerX);
            swerve.setDrivePower(realTimeDrivePower);
            swerve.pid(angle, angle, angle, angle, 0.3);

            //System.out.println(swerve.getTurningEncoderFL());
        }else{
            swerve.setTurningPower(0, 0, 0, 0);
        }

        if(realTimeDrivePower < 0.05){
            swerve.setDrivePower(0);
        }

        
        
    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}