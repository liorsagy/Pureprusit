package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "arm bot demo", group = "ArmBot")
public class ArmBotDemo extends LinearOpMode {

    public void runOpMode(){
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo handServo = hardwareMap.servo.get("hand_servo");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        //gyro.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()){
            gamepad1.setJoystickDeadzone(.1f);
            double joystickAngle180 = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)); // converting the angle we get by the joystick from radians to degrees
            double joystickAngle360 = (joystickAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
            double robotAngle180 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;; // finding the robot angle
            double robotAngle360 = (robotAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
            double finalAngle180 = robotAngle360 + joystickAngle360; // adding the angle of the robot to the desired drive angle from the joystick to get the angle we need to drive in
            double finalAngle360 = (finalAngle180 + 360) % 360; // converting the angle to a range of 0-360 degrees from a range of -180-180 degrees
            double vectorLength = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)); // calculating using trigonometry what is the length of the vector to know the speed we need to drive in
            double xx = Math.cos(Math.toRadians(finalAngle360)) * vectorLength; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
            double yy = -Math.sin(Math.toRadians(finalAngle360)) * vectorLength; // calculating using trigonometry what power we need to use in the x and y axises to drive in the angle of the vector
            double rotation = gamepad1.left_trigger - gamepad1.right_trigger; // reducing the data we get from the right trigger from the left trigger to get the combined angle that we want to turn

            double p1 = -yy+xx+rotation;
            double p2 = -yy-xx+rotation;
            double p3 = yy-xx+rotation;
            double p4 = yy+xx+rotation;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);
            arm.setPower(-gamepad1.right_stick_y);
            if (gamepad1.x) handServo.setPosition(1);
            else if (gamepad1.b) handServo.setPosition(0);
            telemetry.addData("time", getRuntime());
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            //telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
