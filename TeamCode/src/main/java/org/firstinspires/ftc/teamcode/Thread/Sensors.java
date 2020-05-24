package org.firstinspires.ftc.teamcode.Thread;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SharedFunctionality.RobotHardwareMap;

import java.util.Locale;


//@TeleOp(name = "Sensors", group = "TeleOp")
@SuppressWarnings("WeakerAccess")
public class Sensors extends Thread {
    private Variables var;
    private RobotHardwareMap robot;

    private int count = 0;
    private double distanceLAvg;
    private double distanceRAvg;
    private double distanceBAvg;
    private double distanceFMAvg;

    public Sensors(Variables var, RobotHardwareMap robot){
        this.var = var;
        this.robot = robot;
        this.var.setOpModeActive(true);
    }

    @Override
    public void start(){

        //Start the Actual Thread
        run();
    }

    @Override
    public void run(){

        while (var.isOpModeActive()){
            imu();
            distanceSensors();
            colourSensor();
            encoders();
//            setVar();
            var.count ++;
        }
    }

    private void imu() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        var.setHeading(formatAngle(angles.angleUnit, angles.firstAngle));
        var.setRoll(formatAngle(angles.angleUnit, angles.secondAngle));
        var.setPitch(formatAngle(angles.angleUnit, angles.thirdAngle));

        double deltaAngle = angles.firstAngle - var.getLastAngles().firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        var.setGlobalAngle(var.getGlobalAngle() + deltaAngle);

        var.setLastAngles(angles);

    }

    private void encoders() {
        var.setAvgEncoder((robot.leftFront.getCurrentPosition()+
                robot.rightFront.getCurrentPosition()+
                robot.leftBack.getCurrentPosition()+
                robot.rightBack.getCurrentPosition()) / 4);
    }

    private void colourSensor() {

    }

    private void distanceSensors(){
        double distanceLTemp = var.getRobot().distanceL.getDistance(DistanceUnit.MM);
        double distanceRTemp = robot.distanceR.getDistance(DistanceUnit.MM);
        double distanceBTemp = robot.distanceB.getDistance(DistanceUnit.MM);
        double distanceFMTemp = robot.distanceFM.getDistance(DistanceUnit.MM);
        count ++;
        if (count != 5){
            distanceLAvg += distanceLTemp;
            distanceRAvg += distanceRTemp;
            distanceBAvg += distanceBTemp;
            distanceFMAvg += distanceFMTemp;
        }else {
            distanceLAvg /= 5;
            distanceRAvg /= 5;
            distanceBAvg /= 5;
            distanceFMAvg /= 5;
            var.setDistanceL(distanceLAvg);
            var.setDistanceR(distanceRAvg);
            var.setDistanceB(distanceBAvg);
            var.setDistanceFM(distanceFMAvg);

            count = 0;
        }
    }

    private double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
