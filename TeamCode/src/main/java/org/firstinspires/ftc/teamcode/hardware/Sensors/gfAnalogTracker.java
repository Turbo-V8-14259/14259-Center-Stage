package org.firstinspires.ftc.teamcode.hardware.Sensors;
import com.qualcomm.robotcore.hardware.AnalogInput;

//Gluten free meta
public class gfAnalogTracker {
    private AnalogInput encoder;
    private static final double ALL_TELEMETRY_TRACKERS_WRAPAROUND_THRESHOLD = 0.45;
    private static final double LIMIT_TOP = 1;
    private static final double LIMIT_BOT = 0;

    private double resetReading = 0.0f;
    private double currentReading = 0.0f;
    private double lastReading = 0.0f;
    private double lastValidReading = 0.0f;

    private int num_of_wraparounds = 0;

    private boolean wrapingAround = false;

    public gfAnalogTracker(AnalogInput encoder){
        this.encoder = encoder;
        reset();
    }

    public void reset(){
        num_of_wraparounds = 0;
        resetReading = getCurrentReading();
        currentReading = getCurrentReading();
        lastReading = getCurrentReading();
        lastValidReading = getCurrentReading();
    }

    private double getCurrentReading(){
        return Math.abs(encoder.getVoltage()/3.3);
    }

    public void update(){
        currentReading = getCurrentReading();
        double delta_temp = currentReading-lastReading;
        if(Math.abs(delta_temp) > ALL_TELEMETRY_TRACKERS_WRAPAROUND_THRESHOLD){
            wrapingAround = true;
        }else{
            if(wrapingAround){
                wrapingAround = false;
                if(currentReading-lastValidReading >0){
                    num_of_wraparounds --;
                }else{
                    num_of_wraparounds ++;
                }
            }
            lastValidReading = currentReading;
        }
        lastReading = currentReading;
    }
    
    public double getPos(){
        return (num_of_wraparounds+((lastValidReading-resetReading)-LIMIT_BOT)/(LIMIT_TOP-LIMIT_BOT));
    }
    
    public double getRAW(){
        return currentReading;
    }
}
