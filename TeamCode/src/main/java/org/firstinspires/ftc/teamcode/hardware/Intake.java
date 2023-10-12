package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

public class Intake{
    public double intakePower = 0;
    public DcMotorEx intakeMotor;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;

    }

    public Intake stop(){
        this.intakeMotor.setPower(0);
        intakePower = 0;
        return this;
    }
    public boolean isBusy() { return this.intakeMotor.isBusy();}

    public void update() {
        this.intakeMotor.setPower(intakePower);
    }
}
