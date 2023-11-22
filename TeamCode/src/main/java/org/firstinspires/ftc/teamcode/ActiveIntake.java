


//    @Override
//    public void init (){
//            Intake = hardwareMap.get(DcMotor.class, "Intake");
//            IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
//        }


//        @Override
//        public void loop(){
//            if (gamepad1.y && !move) {
//                moveIntake();
//                if (gamepad2.b && !move) {
//                    IntakeServo.setPosition(0.5);
//                }else if (gamepad2.x && !move){
//                    IntakeServo.setPosition(0);
//                }else {
//                    ejectPixel();
//                }
//            }
//
//            public void moveIntake(){
//                double intake = gamepad1.right_trigger;
//                double intake = gamepad2.right_trigger;
//                Intake.setPower(intake*speedMultiplier);
//            }
//
//            public void ejectPixel(){
//                double intake = gamepad2.left_trigger;
//                Intake.setPower(intake*speedMultiplier1);
//            }
//
//        }
