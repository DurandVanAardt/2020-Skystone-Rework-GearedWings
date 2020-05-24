package org.firstinspires.ftc.teamcode.Main.TeamTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.SharedFunctionality.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.Thread.Variables;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class TeleOpTest extends OpMode {

    private Variables var;
    private RobotHardwareMap robot;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        var = new Variables(hardwareMap, var);
        robot = var.getRobot();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        robot.rightFront.setPower(leftPower);
        robot.rightBack.setPower(leftPower);
        robot.leftFront.setPower(rightPower);
        robot.leftBack.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    @Override
    public void stop() {
        var.setOpModeActive(false);
    }

    private void composeTelemetry() {

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getHeading());
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getRoll());
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getPitch());
                    }
                })
                .addData("DistanceFM", var.getDistanceFM())

                .addData("DistanceL", var.getDistanceL())

                .addData("DistanceR", var.getDistanceR())

                .addData("DistanceB", var.getDistanceB())

                .addData("Average Encoder", var.getAvgEncoder());

    }
}
