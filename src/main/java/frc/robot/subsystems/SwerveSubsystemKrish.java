package frc.robot.subsystems;


import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class SwerveSubsystemKrish extends SubsystemBase{

    private SwerveModuleKrish fl, fr, bl, br;
    private double currentPIDTime, previousPIDTime, previousPIDErrorFL, previousPIDErrorFR, previousPIDErrorBL, previousPIDErrorBR, p = 0.1, i = 0, d = 0;
    private boolean flReverse, frReverse, blReverse, brReverse;

    

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
        if(!flReverse){
            fl.setDrivePower(power);
        }else{
            fl.setDrivePower(-power);
        }

        if(!frReverse){
            fr.setDrivePower(-power);
        }else{
            fr.setDrivePower(power);
        }

        if(!blReverse){
            bl.setDrivePower(-power);
        }else{
            bl.setDrivePower(power);
        }

        if(!brReverse){
            br.setDrivePower(-power);
        }else{
            br.setDrivePower(power);
        }
    }

    public void setTurningPower(double flPower, double frPower, double blPower, double brPower){
        fl.setTurningPower(-flPower);
        fr.setTurningPower(-frPower);
        bl.setTurningPower(-blPower);
        br.setTurningPower(-brPower);
    }

    public double wrapAngle(double angle){
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }

        return angle;
    }

    private static double closestAngle(double a, double b){
        // get direction
        double dir = (b % (2 * Math.PI)) - (a % (2 * Math.PI));

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > Math.PI)
        {
                dir = -(Math.signum(dir) * (2 * Math.PI)) + dir;
        }
        return dir;
    }

    public void pid(double flTarget, double frTarget, double blTarget, double brTarget, double limiter){
        currentPIDTime = System.currentTimeMillis();

        // flTarget = wrapAngle(flTarget);
        // frTarget = wrapAngle(frTarget);
        // blTarget = wrapAngle(blTarget);
        // brTarget = wrapAngle(brTarget);


        // double tempFLTarget = wrapAngle(flTarget + Math.PI);
        // double tempFRTarget = wrapAngle(frTarget + Math.PI);
        // double tempBLTarget = wrapAngle(blTarget + Math.PI);
        // double tempBRTarget = wrapAngle(brTarget + Math.PI);


        // double checkingFLError = 0;
        // double checkingFRError = 0;
        // double checkingBLError = 0;
        // double checkingBRError = 0;

        // double checkingEfficientFLError = 0;
        // double checkingEfficientFRError = 0;
        // double checkingEfficientBLError = 0;
        // double checkingEfficientBRError = 0;


        //Jank Math.PI wrapper (FL)
        // if(flTarget < 0){
        //     checkingFLError = (2 * Math.PI + flTarget);
        // }

        // if(flTarget > 0){
        //     checkingFLError = (-2 * Math.PI + flTarget);

        // }

        //Jank Math.PI wrapper (more efficient rotate FL)
        // if(tempFLTarget < 0){
        //     checkingEfficientFLError = (2 * Math.PI + tempFLTarget);
        // }

        // if(tempFLTarget > 0){
        //     checkingEfficientFLError = (-2 * Math.PI + tempFLTarget);

        // }

        //Jank Math.PI wrapper (FR)
        // if(frTarget < 0){
        //     checkingFRError = (2 * Math.PI + frTarget);
        // }

        // if(frTarget > 0){
        //     checkingFRError = (-2 * Math.PI + frTarget);
        // }

        //Jank Math.PI wrapper (more efficient rotate FR)
        // if(tempFRTarget < 0){
        //     checkingEfficientFRError = (2 * Math.PI + tempFRTarget);
        // }

        // if(tempFRTarget > 0){
        //     checkingEfficientFRError = (-2 * Math.PI + tempFRTarget);

        // }


        //Jank Math.PI wrapper (BL)
        // if(blTarget < 0){
        //     checkingBLError = (2 * Math.PI + blTarget);
        // }

        // if(blTarget > 0){
        //     checkingBLError = (-2 * Math.PI + blTarget);
        // }

        //Jank Math.PI wrapper (more efficient rotate BL)
        // if(tempBLTarget < 0){
        //     checkingEfficientBLError = (2 * Math.PI + tempBLTarget);
        // }

        // if(tempBLTarget > 0){
        //     checkingEfficientBLError = (-2 * Math.PI + tempBLTarget);

        // }

        //Jank Math.PI wrapper (BR)
        // if(brTarget < 0){
        //     checkingBRError = (2 * Math.PI + brTarget);
        // }

        // if(brTarget > 0){
        //     checkingBRError = (-2 * Math.PI + brTarget);
        // }

        //Jank Math.PI wrapper (more efficient rotate BR)
        // if(tempBRTarget < 0){
        //     checkingEfficientBRError = (2 * Math.PI + tempBRTarget);
        // }

        // if(tempBRTarget > 0){
        //     checkingEfficientBRError = (-2 * Math.PI + tempBRTarget);

        // }

        
        // if(flTarget == Math.PI || flTarget == -Math.PI){
        //     if(Math.abs(Math.PI - fl.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - fl.getAbsoluteEncoderRad())){
        //         flTarget = -Math.PI;
        //     }else{
        //         flTarget = Math.PI;
        //     }
        // }

        // if(frTarget == Math.PI || frTarget == -Math.PI){
        //     if(Math.abs(Math.PI - fr.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - fr.getAbsoluteEncoderRad())){
        //         frTarget = -Math.PI;
        //     }else{
        //         frTarget = Math.PI;
        //     }
        // }

        // if(blTarget == Math.PI || blTarget == -Math.PI){
        //     if(Math.abs(Math.PI - bl.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - bl.getAbsoluteEncoderRad())){
        //         blTarget = -Math.PI;
        //     }else{
        //         blTarget = Math.PI;
        //     }
        // }

        // if(brTarget == Math.PI || brTarget == -Math.PI){
        //     if(Math.abs(Math.PI - br.getAbsoluteEncoderRad()) > Math.abs(-Math.PI - br.getAbsoluteEncoderRad())){
        //         brTarget = -Math.PI;
        //     }else{
        //         brTarget = Math.PI;
        //     }
        // }



        double flError = closestAngle(fl.getAbsoluteEncoderRad(), flTarget);
        double frError = closestAngle(fr.getAbsoluteEncoderRad(), frTarget);
        double blError = closestAngle(bl.getAbsoluteEncoderRad(), blTarget);
        double brError = closestAngle(br.getAbsoluteEncoderRad(), brTarget);

        //System.out.println("flError: " + flError);
        //System.out.println(flTarget);
        System.out.println(fl.getAbsoluteEncoderRad());


        //System.out.println("frError: " + frError);
        //System.out.println(frTarget);
        //System.out.println(fr.getAbsoluteEncoderRad());

        //System.out.println("blError: " + blError);
        //System.out.println(blTarget);
        //System.out.println(bl.getAbsoluteEncoderRad());

       
        //System.out.println("brError: " + brError);
        //System.out.println(brTarget);
        //System.out.println(br.getAbsoluteEncoderRad());

        // double tempFLError = tempFLTarget - fl.getAbsoluteEncoderRad();
        // double tempFRError = tempFRTarget - fr.getAbsoluteEncoderRad();
        // double tempBLError = tempBLTarget - bl.getAbsoluteEncoderRad();
        // double tempBRError = tempBRTarget - br.getAbsoluteEncoderRad();



        //Check which Error is better based on Jank Wrapper
        // if(Math.abs(checkingFLError) <= Math.abs(flError) && checkingFLError != 0){
        //     flError = checkingFLError;
            
        // }

        // if(Math.abs(checkingFRError) <= Math.abs(frError) && checkingFRError != 0){
        //     frError = checkingFRError;
        // }

        // if(Math.abs(checkingBLError) <= Math.abs(blError) && checkingBLError != 0){
        //     blError = checkingBLError;
        // }

        // if(Math.abs(checkingBRError) <= Math.abs(brError) && checkingBRError != 0){
        //     brError = checkingBRError;
        // }

        //Check which Error is better based on Jank Wrapper (using efficient)
        // if(Math.abs(checkingEfficientFLError) <= Math.abs(tempFLError) && checkingEfficientFLError != 0){
        //     tempFLError = checkingEfficientFLError;
            
        // }

        // if(Math.abs(checkingEfficientFRError) <= Math.abs(tempFRError) && checkingEfficientFRError != 0){
        //     tempFRError = checkingEfficientFRError;
        // }

        // if(Math.abs(checkingEfficientBLError) <= Math.abs(tempBLError) && checkingEfficientBLError != 0){
        //     tempBLError = checkingEfficientBLError;
        // }

        // if(Math.abs(checkingEfficientBRError) <= Math.abs(tempBRError) && checkingEfficientBRError != 0){
        //     tempBRError = checkingEfficientBRError;
        // }


        //Check whether efficient is better of not
        // if(Math.abs(tempFLError) < Math.abs(flError)){
        //     flError = tempFLError;
        //     flReverse = true;
            
        // }else{
        //     flReverse = false;
        // }

        // if(Math.abs(tempFRError) < Math.abs(frError)){
        //     frError = tempFRError;
        //     frReverse = true;
            
        // }else{
        //     frReverse = false;
        // }

        // if(Math.abs(tempBLError) < Math.abs(blError)){
        //     blError = tempBLError;
        //     blReverse = true;
            
        // }else{
        //     blReverse = false;
        // }

        // if(Math.abs(tempBRError) < Math.abs(brError)){
        //     brError = tempBRError;
        //     brReverse = true;
            
        // }else{
        //     brReverse = false;
        // }




        //Integral and derivative aren't doing anything at the moment
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

        //Limiting max power
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

        //Setting previous values
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