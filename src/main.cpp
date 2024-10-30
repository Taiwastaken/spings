#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/extra/widgets/list/lv_list.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <string>
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motor_group({-16, -17, 18}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({13, -14, 15}, pros::MotorGearset::blue);
pros::Motor intake_stage_1(-11, pros::MotorGearset::green);
pros::Motor intake_stage_2(12, pros::MotorGearset::green);
pros::Motor lady_brown(19,pros::MotorGearset::green);
pros::adi::DigitalOut mogo_mech('H');
pros::adi::DigitalOut intake_lift('A');
pros::adi::DigitalOut doinker('B');
pros::Imu imu(10);
pros::Optical color_sorter(1);
int lady_brown_pos = 1;
int counter = 0;
bool color_sorter_red_enable=false;
bool color_sorter_blue_enable=false;
bool counter_enable=false;
bool mogo_mech_extended=false;
bool intake_lift_extended=false;
bool doinker_extended=false;
bool s2diable=false;
lemlib::Drivetrain drivetrain(&left_motor_group, 
                              &right_motor_group, 
                              12.5, 
                              lemlib::Omniwheel::NEW_325, 
                              360, 
                              2); 
lemlib::OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr, 
                            nullptr, 
                            &imu 
);
lemlib::ControllerSettings lateral_controller(20, 
                                              0, 
                                            2, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              30 
);

lemlib::ControllerSettings angular_controller(3, 
                                            0, 
                                              15, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              0 );


lemlib::Chassis chassis(drivetrain, 
                        lateral_controller, 
                        angular_controller, 
                        sensors 
);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(6, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0); 
    lady_brown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD) ;
	pros::lcd::initialize();
    color_sorter.set_led_pwm(50);
	pros::lcd::register_btn1_cb(on_center_button);
    intake_lift.set_value(false);
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); 
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
            pros::lcd::print(3 , "Heading: %f", imu.get_heading()); 
            pros::delay(20);
        }   
    });
}
void disabled() {}
void competition_initialize() {
    intake_lift.set_value(false);
}
void auton_red_far_4ring(){
    lady_brown.move_absolute(500, 200);
    chassis.moveToPoint(0, 20, 1500);
    chassis.turnToHeading(-150, 2000);
    chassis.moveToPoint(8.5, 23, 1500, {false});
    pros::delay(500);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-9, 24, 2000);
    chassis.moveToPoint(-2, 24, 2000, {false});
    chassis.moveToPoint(-9,  46, 2000);
    chassis.moveToPoint(-9,  36, 1500, {false});
    chassis.turnToHeading(-30, 700);
    chassis.moveToPoint(-20, 44, 1500);
    chassis.moveToPoint(0, 34, 3000, {false});
    chassis.turnToHeading(90, 1000);
    intake_lift.set_value(true);
    chassis.moveToPoint(30, 34, 1500);
    pros::delay(1000);
    intake_lift.set_value(false);
}
void auton_blue_far_4ring(){
    lady_brown.move_absolute(500, 200);
    chassis.moveToPoint(0, 20, 1500);
    chassis.turnToHeading(-150, 2000);
    chassis.moveToPoint(8.5, 23, 1500, {false});
    pros::delay(500);
    mogo_mech.set_value(true);
    pros::delay(500);
    intake_stage_1.move_velocity(200);
    intake_stage_2.move_velocity(200);
    chassis.moveToPoint(-10, 24, 2000);
    chassis.moveToPoint(-2, 24, 2000, {false});
    chassis.moveToPoint(10,  46, 2000);
    chassis.moveToPoint(10,  36, 1500, {false});
    chassis.turnToHeading(30, 700);
    chassis.moveToPoint(20, 46, 1500);
    chassis.moveToPoint(0, 10, 3000, {false});
    chassis.turnToHeading(-90, 1000);
    intake_lift.set_value(true);
    chassis.moveToPoint(-30, 10, 1500);
    pros::delay(1000);
    intake_lift.set_value(false);
}
void auton_red_far_SAWP(){

}
void auton_blue_far_SAWP(){

}
void auton_red_near_2ring(){

}
void auton_blue_near_2ring(){

}
void auton_red_near_goalrush(){

}
void auton_blue_near_goalrush(){
    
}
void autonomous() {
    auton_red_far_4ring();
}
void pneumaticsControl(){
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
                mogo_mech_extended = ! mogo_mech_extended;
                mogo_mech.set_value(mogo_mech_extended);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
                intake_lift_extended = ! intake_lift_extended;
                intake_lift.set_value(intake_lift_extended);
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
                doinker_extended = ! doinker_extended;
                doinker.set_value(doinker_extended);
            }
            
}
void lady_brown_control(){
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)&& lady_brown_pos>0){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            lady_brown.set_zero_position(lady_brown.get_position()+10);
        }else{
            lady_brown_pos--;
        }
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)&& lady_brown_pos<4){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            lady_brown.set_zero_position(lady_brown.get_position()-10);
        }else{
            lady_brown_pos++;
        }
	}
	switch(lady_brown_pos){
		case 1:
            lady_brown.move_absolute(0, -200);
			break;
		case 2:
		    lady_brown.move_absolute(225, 200);
			break;
		case 3:
		    lady_brown.move_absolute(1300, 100);
			break;
        case 4:
            lady_brown.move_velocity(lady_brown.get_position()<1500?master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)?200:0:0);
	}
}
void color_sort(){
    if(color_sorter.get_hue()>100&&color_sorter_blue_enable){
            counter_enable=true;
        }
        if(counter_enable){
            if(counter>15){
                counter_enable=false;
                s2diable=true;
            }else{
                counter++;  
            }
        }else if(!counter_enable){
            if(counter<1){
                s2diable=false;
            }else{
                counter--;
            }
            
        }
}
void opcontrol() {
	while (true) {
		pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
        pros::lcd::print(6,"Counter: %d", counter);
        pros::lcd::print(7,"Hue: %f", color_sorter.get_hue());
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		intake_stage_1.move_velocity(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)?200:master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)?-200:0);
        if(!s2diable){
            intake_stage_2.move_velocity(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)?200:master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)?-200:0);
        }else{
            intake_stage_2.move_velocity(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)?0.l:master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)?0:0);
        }
		pneumaticsControl();
		lady_brown_control();
		chassis.arcade(leftY, rightX); 
		pros::delay(20);
	}
}