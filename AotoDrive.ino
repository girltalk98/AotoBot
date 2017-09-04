

//#include <Servo.h>
#include <AFMotor.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C  lcd(0x3F, 16, 2);

AF_DCMotor mtR(4, MOTOR34_64KHZ);
AF_DCMotor mtL(3, MOTOR34_64KHZ);
//Servo radarServo;

#define SERVO_PIN1 9
#define SERVO_PIN2 10

// Human
#define HUMANDETACT_PIN 2

// Ultrasonic
#define ECHO_PIN SERVO_PIN1
#define TRIG_PIN SERVO_PIN2

#define ECHO_PIN2 A0
#define TRIG_PIN2 A1

#define MAX_SPEED 255 // Max 255
#define MAX_SPEED_OFFSET 40 // this sets offset to allow for differences between the two DC traction motors

#define USER_MAX_SPEED (MAX_SPEED - 105) // Max 255

#define LEFT 11
#define RIGHT 12

#define MIN_DIST 40

String motorSet = "";
String strCmd;
int curDist1 = 0;
int curDist2 = 0;
int nSleepStat;


void setup()
{
	//radarServo.attach(SERVO_PIN2, 0, 180);
	//radarServo.write(90);

	lcd.init();                      // initialize the lcd 
	lcd.backlight();

	// 핀 입출력
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN2, INPUT);
	pinMode(TRIG_PIN2, OUTPUT);
	pinMode(HUMANDETACT_PIN, INPUT);
	
	mtR.setSpeed(0);
	mtL.setSpeed(0);

	nSleepStat = HIGH;

	delay(1000);
	Serial.begin(9600);
}



void loop()
{
	delay(30);

	// 움직임이 감지될때만 작동		
	if (digitalRead(HUMANDETACT_PIN) != HIGH) {
		if (nSleepStat != LOW) {
			// LCD 처리 (센서 거리)
			lcd.clear();
			lcd.setCursor(0, 0);	// 첫줄
			lcd.print("(-_-) zzz");
			lcd.setCursor(0, 1);	// 둘째줄
			lcd.print("I am sleepy..");
			nSleepStat = LOW;
		}
		return;
	}
	nSleepStat = HIGH;
	
	// Ultrasonic1 (왼쪽)
	int distance1 = 0;
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);
	distance1 = pulseIn(ECHO_PIN, HIGH) / 58.2; /* 센치미터(cm) */
	
	// Ultrasonic1 (오른쪽)
	int distance2 = 0;
	digitalWrite(TRIG_PIN2, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN2, LOW);
	distance2 = pulseIn(ECHO_PIN2, HIGH) / 58.2; /* 센치미터(cm) */
			
	// 초음파센서 잡음제거 (최대 측정거리 450cm)
	if (distance1 < 450)
		curDist1 = distance1;

	if (distance2 < 450)
		curDist2 = distance2;

	Serial.print("Dist1=");
	Serial.println(curDist1);
	Serial.print("Dist2=");
	Serial.println(curDist2);

	// LCD 처리 (센서 거리)
	lcd.clear();
	lcd.setCursor(0, 0);	// 첫줄
	lcd.print(curDist2);
	lcd.print("----");
	lcd.print(curDist1);

	
	if (curDist1 > MIN_DIST && curDist2 > MIN_DIST)
	{
		moveStraight(FORWARD);		
		strCmd = "GO!! Go!! Go!!";
	}
	else// if (curDist1 <= MIN_DIST || curDist2 <= MIN_DIST)
	{
		if (curDist1 >= curDist2)
		{
			moveOffset(RIGHT);
			Serial.println("RIGHT");
			strCmd = "====>";
		}
		else
		{
			moveOffset(LEFT);
			Serial.println("LEFT");			
			strCmd = "<====";
		}
	}

	lcd.setCursor(0, 1);	// 둘째줄
	lcd.print(strCmd);
	
	if (Serial.available() > 0)
	{	
		String strCommand;
		strCommand = Serial.readString();
		
		Serial.println(strCommand);
		char rvChar = strCommand[0];
		Serial.println((int)rvChar);
		switch (rvChar)
		{
		case 'w':
			// 너무 가까우면 
			if (curDist1 <= 10 || curDist2 <= 10) {
				moveStraight(FORWARD);
				Serial.println("FORWARD");
			}
			else {
				Serial.println("BLOCKED!!FORWARD");
			}			
			break;
		case 's':
			moveStraight(BACKWARD);
			Serial.println("BACKWARD");
			break;
		case 'a':
			moveOffset(LEFT);
			Serial.println("LEFT");
			break;
		case 'd':
			moveOffset(RIGHT);
			Serial.println("RIGHT");
			break;
		default:
			moveStop();
			Serial.println("STOP");
			break;
		}
	}

	

}

void moveStop() {
	mtR.run(RELEASE);
	mtL.run(RELEASE);
}  // stop the motors.

void moveStraight(uint8_t dir) {
	motorSet = (dir == FORWARD ? "FORWARD" : "BACKWARD");
	mtR.run(dir);      // turn it on going forward
	mtL.run(dir);      // turn it on going forward
	for (int speedSet = 70; speedSet <= USER_MAX_SPEED; speedSet += 10) // slowly bring the speed up to avoid loading down the batteries too quickly
	{
		mtR.setSpeed(speedSet);
		mtL.setSpeed(speedSet);
		delay(20);
	}
	moveStop();
}

void moveOffset(uint8_t dir) {
	mtR.setSpeed(USER_MAX_SPEED);
	mtL.setSpeed(USER_MAX_SPEED);

	motorSet = (dir == LEFT ? "LEFT" : "RIGHT");
	mtR.run(dir == LEFT ? FORWARD : BACKWARD);      // turn it on going forward
	mtL.run(dir == LEFT ? BACKWARD : FORWARD);      // turn it on going forward

	delay(100);
	moveStop();
}
