package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake{
    public double intakePower = 0;
    public DcMotorEx intakeMotor;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;

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
