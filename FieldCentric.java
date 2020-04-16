package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.purePersuit.RobotMonement;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

@TeleOp(name = "field centric")
public class FieldCentric extends OpMode {

    DcMotor lf, rf, lb, rb;
    BNO055IMU imu;

    private static double x = 0;
    private static double y = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        lb = hardwareMap.dcMotor.get("back_left_motor");
        lf = hardwareMap.dcMotor.get("front_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        rb = hardwareMap.dcMotor.get("back_right_motor");
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        gamepad1.setJoystickDeadzone(.1f);
        double joystickAngle180 = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)); // converting the angle we get by the joystick from radians to degrees
        double joystickAngle360 = (joystickAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double robotAngle180 = getAngle(); // finding the robot angle
        double robotAngle360 = (robotAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double finalAngle180 = robotAngle360 + joystickAngle360; // adding the angle of the robot to the desired drive angle from the joystick to get the angle we need to drive in
        double finalAngle360 = (finalAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
        double vectorLength = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)); // calculating using trigonometry what is the length of the vector to know the speed we need to drive in
        double xx = Math.cos(Math.toRadians(finalAngle360)) * vectorLength; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
        double yy = -Math.sin(Math.toRadians(finalAngle360)) * vectorLength; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
        double rotation = gamepad1.left_trigger - gamepad1.right_trigger; // reducing the data we get from the right trigger from the left trigger to get the combined angle that we want to turn

        double lfp = yy + xx - rotation;
        double rfp = yy - xx + rotation;
        double lbp = yy - xx - rotation;
        double rbp = yy + xx + rotation;
        lb.setPower(lbp);
        lf.setPower(lfp);
        rf.setPower(rfp);
        rb.setPower(rbp);

        RobotMonement.setWorldPosition(lf,  rf,  lb,  rb,  imu);

        telemetry.addData("gamepad x", gamepad1.left_stick_x);
        telemetry.addData("gamepad y", gamepad1.left_stick_y);
        telemetry.addData("rotation", rotation);
        telemetry.addData("x", RobotMonement.worldXPosition);
        telemetry.addData("y", RobotMonement.worldYPosition);
        telemetry.addData("angle", RobotMonement.getAngle(imu));
    }

    private double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


}