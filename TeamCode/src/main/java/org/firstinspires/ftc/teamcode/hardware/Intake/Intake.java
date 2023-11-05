package org.firstinspires.ftc.teamcode.hardware.Intake;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

public class Intake{
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = 1;
    public double intakePower = 0;
    public DcMotorEx intakeMotor;
    //NOT A DcMotorBetter, just a DcMotorEx

    public ServoMotorBetter intakePivot;

    public Intake(DcMotorEx intakeMotor, ServoMotorBetter intakePivot) {
        this.intakeMotor = intakeMotor;
        this.intakePivot = intakePivot;
        this.intakePivot.setLowerBound(LOWER_BOUND);
        this.intakePivot.setUpperBound(UPPER_BOUND);
    }
    public Intake stop(){
        intakePower = 0;
        return this;
    }
    public Intake setPower(double p){
        intakePower = p;
        return this;
    }
    public boolean isBusy() { return this.intakeMotor.isBusy();}
    public void update() {
        this.intakeMotor.setPower(intakePower);
    }

}
