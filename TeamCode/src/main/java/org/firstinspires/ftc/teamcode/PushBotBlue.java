//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous
//public class PushBotAutoRed extends LinearOpMode {
//        HardwarePowerpuffs robot = new HardwarePowerpuffs();
//        @Override
//        public void runOpMode() throws InterruptedException {
//                telemetry.addData("teamPropLocations", teamPropLocations);
//                telemetry.update();
//                moveBackward(0.3, 46);
//                robot.Wrist.setPosition(0.08);//down
//                robot.ClawL.setPosition(0.3);//open
//                sleep(20);
//                robot.Wrist.setPosition(0.4);//up
//                robot.ClawL.setPosition(0.08);//close
//
//                turnRight(0.3, 14.5);
//                moveBackward(0.3, 20);
//                strafeLeft(0.3, 22);
//                moveForward(0.3, 10);
//                dropPurplePixelDone = true;
//        }
//}
