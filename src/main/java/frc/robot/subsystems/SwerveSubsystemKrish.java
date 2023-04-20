package frc.robot.subsystems;


import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystemKrish extends SubsystemBase{

    private SwerveModuleKrish fl, fr, bl, br;
    private double currentPIDTime, previousPIDTime, previousPIDErrorFL, previousPIDErrorFR, previousPIDErrorBL, previousPIDErrorBR, p = 0.1, i = 0, d = 0;

    

    public SwerveSubsystemKrish(){
        fl = new SwerveModuleKrish(Constants.flDriveId, Constants.flTurnId, false, false, 1, 0, false);
        fr = new SwerveModuleKrish(Constants.frDriveId, Constants.frTurnId, false, false, 2, 0, false);
        bl = new SwerveModuleKrish(Constants.blDriveId, Constants.blTurnId, false, false, 4, 0, false);
        br = new SwerveModuleKrish(Constants.brDriveId, Constants.brTurnId, false, false, 3, 0, false);

        setMode(IdleMode.kBrake);
    
    }

    public double getTurningEncoderFL(){
        return fl.getTurningPosition();
    }

    public void setMode(IdleMode mode){
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    public void setDrivePower(double power){
        fl.setDrivePower(power);
        fr.setDrivePower(power);
        bl.setDrivePower(power);
        br.setDrivePower(power);
    }

    public void setTurningPower(double flPower, double frPower, double blPower, double brPower){
        fl.setTurningPower(-flPower);
        fr.setTurningPower(-frPower);
        bl.setTurningPower(-blPower);
        br.setTurningPower(-brPower);
    }

    private double wrapAngle(double angle){
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }

        return angle;
    }

    public void pid(double flTarget, double frTarget, double blTarget, double brTarget, double limiter){
        currentPIDTime = System.currentTimeMillis();

        flTarget = wrapAngle(flTarget);
        frTarget = wrapAngle(frTarget);
        blTarget = wrapAngle(blTarget);
        brTarget = wrapAngle(brTarget);

        if(flTarget == Math.PI || flTarget == -Math.PI){
            if(Math.abs(Math.PI - fl.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - fl.getAbsoluteEncoderRad())){
                flTarget = -Math.PI;
            }else{
                flTarget = Math.PI;
            }
        }

        if(frTarget == Math.PI || frTarget == -Math.PI){
            if(Math.abs(Math.PI - fr.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - fr.getAbsoluteEncoderRad())){
                frTarget = -Math.PI;
            }else{
                frTarget = Math.PI;
            }
        }

        if(blTarget == Math.PI || blTarget == -Math.PI){
            if(Math.abs(Math.PI - bl.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - bl.getAbsoluteEncoderRad())){
                blTarget = -Math.PI;
            }else{
                blTarget = Math.PI;
            }
        }

        if(brTarget == Math.PI || brTarget == -Math.PI){
            if(Math.abs(Math.PI - br.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - br.getAbsoluteEncoderRad())){
                brTarget = -Math.PI;
            }else{
                brTarget = Math.PI;
            }
        }



        double flError = flTarget - fl.getAbsoluteEncoderRad();
        double frError = frTarget - fr.getAbsoluteEncoderRad();
        double blError = blTarget - bl.getAbsoluteEncoderRad();
        double brError = brTarget - br.getAbsoluteEncoderRad();



        double flIntegral = 0.5 * (flError + previousPIDErrorFL) * (currentPIDTime - previousPIDTime);
        double frIntegral = 0.5 * (frError + previousPIDErrorFR) * (currentPIDTime - previousPIDTime);
        double blIntegral = 0.5 * (blError + previousPIDErrorBL) * (currentPIDTime - previousPIDTime);
        double brIntegral = 0.5 * (brError + previousPIDErrorBR) * (currentPIDTime - previousPIDTime);

        double flDerivative = (flError - previousPIDErrorFL)/(currentPIDTime - previousPIDTime);
        double frDerivative = (frError - previousPIDErrorFR)/(currentPIDTime - previousPIDTime);
        double blDerivative = (blError - previousPIDErrorBL)/(currentPIDTime - previousPIDTime);
        double brDerivative = (brError - previousPIDErrorBR)/(currentPIDTime - previousPIDTime);


        double flPower = ((p * flError) + (i * flIntegral) + (d * flDerivative));
        double frPower = ((p * frError) + (i * frIntegral) + (d * frDerivative));
        double blPower = ((p * blError) + (i * blIntegral) + (d * blDerivative));
        double brPower = ((p * brError) + (i * brIntegral) + (d * brDerivative));

        if(Math.abs(flPower) > limiter){
            if(flPower < 0){
                flPower = -1 * limiter;
            }else{
                flPower = limiter;
            }
        }

        if(Math.abs(frPower) > limiter){
            if(frPower < 0){
                frPower = -1 * limiter;
            }else{
                frPower = limiter;
            }
        }

        if(Math.abs(blPower) > limiter){
            if(blPower < 0){
                blPower = -1 * limiter;
            }else{
                blPower = limiter;
            }
        }

        if(Math.abs(brPower) > limiter){
            if(brPower < 0){
                brPower = -1 * limiter;
            }else{
                brPower = limiter;
            }
        }



        setTurningPower(flPower, frPower, blPower, brPower);

        previousPIDTime = currentPIDTime;
        previousPIDErrorFL = flError;
        previousPIDErrorFR = frError;
        previousPIDErrorBL = blError;
        previousPIDErrorBR = brError;





    }

    public void changeOrienation(boolean toTurn){
        if(toTurn){
            pid(-Math.PI / 4, Math.PI / 4, Math.PI / 4, -Math.PI / 4, 0.5);
        }else{
            pid(0,0,0,0, 0.5);
        }
    }

    

    @Override
    public void periodic(){

    }

}