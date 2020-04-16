package org.firstinspires.ftc.teamcode.purePersuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class RobotMonement{

    public static double worldXPosition = 0;
    public static double worldYPosition = 0;
    static double worldAngle_rad = 0;
    private static double x = 0;
    private static double y = 0;
    private static int  LF;
    private static int  RF;
    private static int  LB;
    private static int  RB;


    //hi

    public static CurvePoint followCurve(ArrayList<CurvePoint> allPoints, double followAngle,
                                   DcMotor lf,
                                   DcMotor rf,
                                   DcMotor lb,
                                   DcMotor rb,
                                   BNO055IMU imu){

        RobotMonement.setWorldPosition(lf,rf,lb,rb,imu);
        CurvePoint followMe = getFollowPointPath(allPoints , new point(worldXPosition, worldYPosition), allPoints.get(0).followDistance, lf,rf,lb,rb,imu);
        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed,lf,rf,lb,rb,imu);
        return followMe;
    }

    public static CurvePoint getFollowPointPath(ArrayList< CurvePoint> pathPoints, point robotlocation, double followRadius, DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, BNO055IMU imu){
         CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int i = 0 ; i < pathPoints.size()-1 ; i ++ ) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get( i +1 );

            ArrayList<point> intersections  = MathFunction.lineCircleintersaction(robotlocation , followRadius , startLine.toPoint(), endLine.toPoint());


            double closetAngle = 100000000;
            for (point thisIntersections : intersections){
                double angle = Math.atan2(thisIntersections.y - worldYPosition, thisIntersections.x - worldYPosition);
                double deltaAngle = Math.abs(MathFunction.AngelWarp(angle - worldAngle_rad));
                if ( deltaAngle < closetAngle){
                    closetAngle = deltaAngle;
                    followMe.setPoint(thisIntersections);
                }
            }

        }
        return followMe;
    }






    public static void  goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed,
                                     DcMotor lf,
                                     DcMotor rf,
                                     DcMotor lb,
                                     DcMotor rb,
                                     BNO055IMU imu){

        RobotMonement.setWorldPosition(lf,rf,lb,rb,imu);

        double distanceToTarget = Math.hypot(x- worldXPosition, y- worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y- worldYPosition, x- worldXPosition);

        double relativeAngleToPoint = MathFunction.AngelWarp(absoluteAngleToTarget - ( worldAngle_rad- Math.toRadians(90)));
        double relativeXtoPoint = cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double movementYPower = relativeYtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));

        double movement_x = movementXPower * movementSpeed;
        double movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        double movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(10),-1,1) * turnSpeed;
        if (distanceToTarget < 50){
            movement_y = 0;
            movement_x = 0;
            movement_turn = 0;
        }

        double joystickAngle180 = Math.toDegrees(Math.atan2(movement_x, movement_y)); // converting the angle we get by the joystick from radians to degrees
        double joystickAngle360 = (joystickAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double robotAngle180 = Math.toDegrees(getAngle(imu)); // finding the robot angle
        double robotAngle360 = (robotAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double finalAngle180 = robotAngle360 + joystickAngle360; // adding the angle of the robot to the desired drive angle from the joystick to get the angle we need to drive in
        double finalAngle360 = (finalAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double xx = Math.cos(Math.toRadians(finalAngle360)) * distanceToTarget; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
        double yy = -Math.sin(Math.toRadians(finalAngle360)) * distanceToTarget; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
        double rotation = movement_turn; // reducing the data we get from the right trigger from the left trigger to get the combined angle that we want to turn

        double lfp = yy + xx - rotation;
        double rfp = yy - xx + rotation;
        double lbp = yy - xx - rotation;
        double rbp = yy + xx + rotation;
        lb.setPower(lbp);
        lf.setPower(lfp);
        rf.setPower(rfp);
        rb.setPower(rbp);

    }

    public static void setWorldPosition(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, BNO055IMU imu){
        y = ((lf.getCurrentPosition()-LF) + (rf.getCurrentPosition()-RF) + (lb.getCurrentPosition()-LB) + (rb.getCurrentPosition()-RB))/4;
        x = (((lf.getCurrentPosition()-LF) + (rb.getCurrentPosition()-RB)) - ((rf.getCurrentPosition()-RF) + (lb.getCurrentPosition()-LB)))/4;
        worldXPosition += (cos(getAngle(imu)) * x) + (sin(getAngle(imu)) * y);
        worldYPosition += (cos(getAngle(imu)) * y) + (sin(getAngle(imu)) * x);
        worldAngle_rad = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        LF = lf.getCurrentPosition();
        RF = rf.getCurrentPosition();
        LB = lb.getCurrentPosition();
        RB = rb.getCurrentPosition();
    }

    public static double getAngle(BNO055IMU imu){
       return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}
