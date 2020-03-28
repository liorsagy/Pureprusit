package org.firstinspires.ftc.teamcode.teamcode;
import android.util.Range;

import  org.firstinspires.ftc.teamcode.teamcode.MathFunction;
public class RobotMonement{

    public static  void  goToPosition(double x,double y , double momementSpeed, double preferredAngle, double turnSpeed){

        double worldXPosition = 0;
        double worldYPosition = 0;
        double worldAngle_rad = Math.toRadians(-180);

        double distanceToTarget = Math.hypot(x- worldXPosition, y- worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y- worldYPosition, x- worldXPosition);

        double relativeAngleToPoint = MathFunction.AngelWarp(absoluteAngleToTarget - ( worldAngle_rad- Math.toRadians(90)));
        double relativeXtoPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYtoPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double movementYPower = relativeYtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));

        double movement_x = movementXPower * momementSpeed;
        double movement_y = movementYPower * momementSpeed;;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        double movement_turn = Range.Clip(relativeTurnAngle / Math.toRadians(30),-1,1) * turnSpeed;
        if (distanceToTarget< 10){
            movement_turn = 0;
        }

    }
}
