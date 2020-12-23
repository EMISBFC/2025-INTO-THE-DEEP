package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

@TeleOp(name = "Teleop", group = "DemoBot")
public class Teleop extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        waitForStart();
        boolean aPressed=false;
        boolean yPressed=false;
        boolean upPressed=false;
        boolean downPressed=false;
        double angleSpeed=1;
        while(opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            robot.drive(gamepad1.right_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.addData("angle speed",angleSpeed);
            telemetry.update();

            if (gamepad1.right_trigger<.01) {


                robot.setIntakePower(-gamepad1.left_trigger);

            }
            else
            {

                robot.setIntakePower(gamepad1.right_trigger);

            }


            //Setting power for intake to the right trigger



            //sets flywheel power to the left trigger
            robot.setFlyWheelPower(gamepad2.right_trigger);

            //makes the flywheel rotation servo move with b and x
            if(gamepad2.b)
            {

                robot.flywheelRotateServoLeft.setPower(angleSpeed);

            }

            else if(gamepad2.x)
            {

                robot.flywheelRotateServoLeft.setPower(-angleSpeed);

            }

            else
            {

                robot.flywheelRotateServoLeft.setPower(0);

            }

            if(gamepad2.a&&!aPressed)
            {
                robot.flickRing();
                aPressed=true;
            }
            if(!gamepad2.a)
            {

                aPressed=false;

            }
            if(gamepad2.y&&!yPressed)
            {
                robot.flickRing();
                robot.flickRing();
                robot.flickRing();
                yPressed=true;
            }
            if(!gamepad2.y)
            {

                yPressed=false;

            }

            if(gamepad2.dpad_up&&!upPressed)
            {
                if(angleSpeed<1)
                    angleSpeed+=.1;
                upPressed=true;
            }
            if(!gamepad2.dpad_up)
            {

                upPressed=false;

            }

            if(gamepad2.dpad_down&&!downPressed)
            {
                if(angleSpeed>.11)
                    angleSpeed-=.1;
                downPressed=true;
            }
            if(!gamepad2.dpad_down)
            {

                downPressed=false;

            }

        }

        ThreadPool.renewPool();

    }

}