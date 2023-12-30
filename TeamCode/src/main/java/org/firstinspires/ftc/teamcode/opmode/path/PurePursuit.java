package org.firstinspires.ftc.teamcode.opmode.path;

import static org.firstinspires.ftc.teamcode.opmode.path.PurePursuitUtil.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.opmode.path.PurePursuitUtil.*;
import org.opencv.core.Point;

import java.util.ArrayList;
public class PurePursuit extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double lookaheadDist = 5;
        Point robotLocation = new Point(0,0);
        ArrayList<Point> wayPoints = new ArrayList<>();
        wayPoints.add(robotLocation.clone());
        wayPoints.add(new Point(0, 50));
        wayPoints.add(new Point(23, 70));
        wayPoints.add(new Point(90, 30));
        wayPoints.add(new Point(-80, -60));

        for(int i =0; i<wayPoints.size()-1;++i){
            while(!passedWayPt(robotLocation, wayPoints.get(i+1), lookaheadDist)){
                Point follow = followMe(wayPoints.get(i), wayPoints.get(i+1), robotLocation, lookaheadDist);
                //goTo(follow)
                //update()
            }
        }
    }
}
