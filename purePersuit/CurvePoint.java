package org.firstinspires.ftc.teamcode.purePersuit;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLenght;
    public double slowDownTurnRdians;
    public double slowedDownTurnAmount;


    public CurvePoint( double x, double  y, double moveSpeed, double turnSpeed,
     double followDistance, double slowDownTurnRdians , double slowedDownTurnAmount){

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRdians = slowDownTurnRdians;
        this.slowedDownTurnAmount = slowedDownTurnAmount;
    }

    public CurvePoint( CurvePoint thispoint){
        x = thispoint.x;
        y = thispoint.y;
        moveSpeed = thispoint.moveSpeed;
        turnSpeed = thispoint.turnSpeed;
        followDistance = thispoint.followDistance;
        pointLenght = thispoint.pointLenght;
        slowDownTurnRdians = thispoint.slowDownTurnRdians;
        slowedDownTurnAmount = thispoint.slowedDownTurnAmount;
    }
    public point toPoint(){
        return new point(x,y);
    }
    public void  setPoint(point point){
        x = point.x;
        y = point.y;
    }
}
