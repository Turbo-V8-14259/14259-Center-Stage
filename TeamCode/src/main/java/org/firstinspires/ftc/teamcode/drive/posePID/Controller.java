package org.firstinspires.ftc.teamcode.drive.posePID;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Controller {
    DriveTrain localizer;
    public Controller() {

    }

    public void update() {
        localizer.update();
    }
}
