package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystemKrish extends SubsystemBase{

    private SwerveModuleKrish fl, fr, bl, br;
    private double currentPIDTime, previousPIDTime, previousPIDErrorFL, previousPIDErrorFR, previousPIDErrorBL, previousPIDErrorBR, p, d;

    public SwerveSubsystemKrish(){
        fl = new SwerveModuleKrish(Constants.flDriveId, Constants.flTurnId, false, false, 0, Constants.flAbsoluteId, false);
        fr = new SwerveModuleKrish(Constants.frDriveId, Constants.frTurnId, false, false, 0, 0, false);
        bl = new SwerveModuleKrish(Constants.blDriveId, Constants.blTurnId, false, false, 0, 0, false);
        br = new SwerveModuleKrish(Constants.brDriveId, Constants.brTurnId, false, false, 0, 0, false);

    
    }

    public void setDrivePower(double power){
        fl.setDrivePower(power);
        fr.setDrivePower(power);
        bl.setDrivePower(power);
        br.setDrivePower(power);
    }

    public void setTurningPower(double flPower, double frPower, double blPower, double brPower){
        fl.setTurningPower(flPower);
        fr.setTurningPower(frPower);
        bl.setTurningPower(blPower);
        br.setTurningPower(brPower);
    }

    public void pid(double flTarget, double frTarget, double blTarget, double brTarget, double limiter){
        currentPIDTime = System.currentTimeMillis();

        double flError = flTarget - fl.getAbsoluteEncoderRad();
        double frError = frTarget - fr.getAbsoluteEncoderRad();
        double blError = blTarget - bl.getAbsoluteEncoderRad();
        double brError = brTarget - br.getAbsoluteEncoderRad();

        double flDerivative = (flError - previousPIDErrorFL)/(currentPIDTime - previousPIDTime);
        double frDerivative = (frError - previousPIDErrorFR)/(currentPIDTime - previousPIDTime);
        double blDerivative = (blError - previousPIDErrorBL)/(currentPIDTime - previousPIDTime);
        double brDerivative = (brError - previousPIDErrorBR)/(currentPIDTime - previousPIDTime);


        double flPower = ((p * flError) + (d * flDerivative));
        double frPower = ((p * frError) + (d * frDerivative));
        double blPower = ((p * blError) + (d * blDerivative));
        double brPower = ((p * brError) + (d * brDerivative));

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