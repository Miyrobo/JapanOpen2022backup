#include "include.h"

void checkping();
void standby();
void positionmonitor();
void print_voltage();
void get_sensors();
void cameramonitor();
void cameramonitorold();
void kicker();
void pwm_out();
int cal_line();
int check_line();
int check_line_old();
void neo_dir(int dir, int R, int G, int B);
int get_ball();
void motor_move2(int p1, int p2, int p3, int p4);
void motor_move(int p1, int p2, int p3, int p4);
void dircontrol();
void dircontrol2();
void cal_motorspeed(int D, int Speed);
unsigned long get_timer(int n);
void clr_timer(int n);
int get_dir();
void bno_setup();
void neopixel_setup();
void dinogame();
void lineUI(int,int,int);
void kickertest();
void music();
void set_volume();
void get_pixy();


bool keeperon = 1; //白だけキーパー

int mamorimode = 0; //通信キーパー
int kadomode = 0;

int goalcolor;

int FPS;

int keepermode = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// This is the main Pixy object
Pixy2 pixy;

#define OLED_RESET 4
Adafruit_SSD1306 display(128, 64, &Wire, -1);

int lx = 0, ly = 0;



#define SE_ON


unsigned long S_timer[timer_MAX] = {0};

GOAL seme;
GOAL mamori;
MOTOR mot;


Output motor[] = {Pin_motor1F, Pin_motor1B, Pin_motor2F, Pin_motor2B, Pin_motor3F, Pin_motor3B, Pin_motor4F, Pin_motor4B}; //出力ピンに設定

int ifkeeper = 0;
int line[16];   //外側 前,右,下,左の順に 0,1,2,3  内側 同じく 4,5,6,7
int dir, Dir;
int roll;
int dir_move;  //進行方向
int bd;

int ver39 = 0;

int n5 = 0;
int ping;

bool AA = 0;
bool BB = 0;

int lineF[4];

int battery;

int Speed = MAXspeed;

int Robotn;

int m_power[4] = {0}; //モーター速さ -100~100 左前から時計回りに 0,1,2,3

int linex = 0, liney = 0; //ライン上の位置

//int ball.distance;  //ボールの値の合計

int LINEd;

int kickerf = 0;
int MODE = 0;

int lite = 0;

int conect = 0;

int corner = 0;


//---------------------------------------------------------------------------
SoftwareSerial mySoftwareSerial(45, 41); // IO45をRX, IO41をTXとしてアサイン
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
//---------------------------------------------------------------------------

int Gyro_X = EEPROM[0] * 256 + EEPROM[1] - 3000;
int Gyro_Y = EEPROM[2] * 256 + EEPROM[3] - 3000;
int Gyro_Z = EEPROM[4] * 256 + EEPROM[5] - 3000;
int Accel_X = EEPROM[6] * 256 + EEPROM[7] - 3000;
int Accel_Y = EEPROM[8] * 256 + EEPROM[9] - 3000;
int Accel_Z = EEPROM[10] * 256 + EEPROM[11] - 3000;

BALL ball;

char str[32];

#define MODE_MAX 7

int freq[4] = {1047, 1175, 1319, 1396};

void sound(int hz, int l) {
  tone(buzzer, hz, l * 0.6);
  delay(l);
}

void doremi() {
  for (int i = 0; i < 4; i++) {
    sound(freq[i], 100);
    sound(freq[i], 100);
  }
  for (int i = 2; i >= 0; i--) {
    sound(freq[i], 200);
  }
}

void start_sound() {
  mot.stop();
  if (seme.color == 2) {
    tone(buzzer, 3135, 150); //さくさく
    delay(120);
    tone(buzzer, 987, 50);
    delay(80);
    tone(buzzer, 1174, 50);
    delay(100);
    tone(buzzer, 1046, 50);
    delay(100);
  } else {
    tone(buzzer, 2637, 150); //今の
    delay(120);
    tone(buzzer, 987, 50);
    delay(80);
    tone(buzzer, 1479, 50);
    delay(100);
    tone(buzzer, 1244, 50);
    delay(100);
  }
}

void print_voltage() {
  battery = analogRead(A9);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(80, 0);
  if (battery % 103 < 10) {
    sprintf(str, "%d.0%d V", (battery / 103), (battery % 103));
  } else {
    sprintf(str, "%d.%d V", (battery / 103), (battery % 103));
  }
  display.print(str);
}

int VOLUME = 30;

double ave = 0;
void setup() {
  Wire.begin();

  mySoftwareSerial.begin(9600);
  Serial.begin(9600);
  //
  if (!myDFPlayer.begin(mySoftwareSerial)) {  // DFPlayerを初期化します。USBピンを使ってなければ、デバイスはSD(TF)カードが選択されます
    // 2秒以内に初期化できなかった場合はエラーメッセージを表示
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }

  myDFPlayer.volume(VOLUME);  // ボリュームをセット、 ボリュームは0から30の値で指定可能
  myDFPlayer.play(7); //起動
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize I2C addr to 0x3C ( for 128x64 Display )
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15, 28);
  display.print("GUJO ROBO");
  print_voltage();
  display.display();
  pixy.init();

  Robotn = EEPROM[60];
  bno_setup();
  neopixel_setup();
  pinMode(Pin_kicker, OUTPUT);
  pinMode(Pin_hold, INPUT);
  pinMode(Pin_in1, INPUT);
  pinMode(Pin_in2, INPUT);
  pinMode(Pin_in3, INPUT);
  pinMode(Pin_in4, INPUT);
  pinMode(Pin_out1, OUTPUT);
  pinMode(Pin_out2, OUTPUT);
  pinMode(Pin_out3, OUTPUT);
  pinMode(Pin_out4, OUTPUT);
  digitalWrite(Pin_kicker, LOW);
  pinMode(Pin_BUSY, INPUT);

  // TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31.37255 [kHz]
  // TCCR3B = (TCCR3B & 0b11111000) | 0x01; //31.37255 [kHz]
  // TCCR4B = (TCCR4B & 0b11111000) | 0x01; //31.37255 [kHz]

  clr_timer(0);
  int f = 0;
  //int c = 0;
  if (SW3 == 0) {
    while (get_timer(0) < 1000) {
      for (int i = 0; i < NUM_LEDS; i++) {
        if (get_timer(0) > i * 50)
          leds[i].setHue(int(((255 / NUM_LEDS * i)) % 255));
      }

      if (SW3 == 1) {
        f = 1;
      }


      if (f == 0 && SW3 == 0) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(255, 255, 0);
        }
      } else {
        for (int i = 0; i < NUM_LEDS; i++) {
          //leds[i] = CRGB(255, 0, 0);
        }
      }
      FastLED.show();
    }
  }

  if (f == 0 && SW3 == 0) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    dinogame();
  }
  /*for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 0, 0);
    }
    FastLED.show();
  */
  if (EEPROM.read(50) == 3) {
    goalcolor = 3;
    seme.color = 3;
    mamori.color = 2;
  } else {
    goalcolor = 2;
    seme.color = 2;
    mamori.color = 3;
  }
  for (int i = 0; i < 4; i++) {
    //lineF[i] = EEPROM.read(20 + i);
  }
  if (Robotn == 1) {
    lineF[0] = 150;
    lineF[1] = 280;
    lineF[2] = 280;
    lineF[3] = 280;
  } else {
    lineF[0] = 200;
    lineF[1] = 50;
    lineF[2] = 100;
    lineF[3] = 150;
  }
  start_sound();
  //EEPROM.write(60,2);
  //doremi();//start_sound();
  //buz.tondemowonders();
  //myDFPlayer.play(5);
  //tone(buzzer, 1661, 50);
  //delay(100);
  Dir = get_dir();
  VOLUME = 20;


  myDFPlayer.play(8); //スタート
  standby();

}



void checkping() {
  if (n5 == digitalRead(Pin_in2)) {
    digitalWrite(Pin_out2, !n5);
    n5 = !n5;
    ping = get_timer(6);
    clr_timer(6);
    ave = ave * 0.85 + ping * 0.15;

  }
}

void standby() {
  linex = 0;
  liney = 0;

  while (!TS || MODE == 7) { //右スイッチが倒されるまで待機
    if (!SW1) { //左ボタンで方位リセット
      Dir = get_dir();
      display.clearDisplay();
      display.setTextSize(2);
      if (goalcolor == 2) {
        display.setCursor(20, 25);
        display.print("Yellow");

        tone(buzzer, 1438, 100);
      } else {
        display.setCursor(40, 25);
        display.print("Blue");

        tone(buzzer, 1077, 100);
      }
      display.display();

      delay(100);

      while (!SW1);
    }
    display.clearDisplay();
    digitalWrite(Pin_kicker, LOW);

    motor_move2(1000, 1000, 1000, 1000);


    if (MODE < 3)print_voltage();
    //Serial.println(str);


    get_sensors();

    if (MODE == 0) {
      if (get_timer(45) > 50) {
        clr_timer(45);
        Serial.print("B");
        for (int i = 0; i < 8; i++) {
          Serial.print(1000 - ball.value[i]);
          Serial.print(",");
        }
        Serial.println("");
      }
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }
      int bb = get_ball();
      display.drawCircle(40, 32, 25, WHITE);
      if (bb != 1000)display.fillCircle(40 + sin(bb / 57.3) * ((ball.distance / 100) - 26) * 1.2, 32 - cos(bb / 57.3) * ((ball.distance / 100) - 26) * 1.2, 3, WHITE);

      if (bb != 1000)
        display.drawLine(40, 32, 40 + sin(bb / 57.3) * ((ball.distance / 100) - 26) * 1.2, 32 - cos(bb / 57.3) * ((ball.distance / 100) - 26) * 1.2, WHITE);

      display.drawLine(40, 32 + 32, 40, 32 - 32, WHITE);
      display.drawLine(40 - 32, 32, 40 + 32, 32, WHITE);

      int n = 16;
      for (int i = 0; i < n; i++) {
        display.drawLine(40 + sin(i * 360 / n / 57.3) * 27, 32 + cos(i * 360 / n / 57.3) * 27, 40 + sin(i * 360 / n / 57.3) * 23, 32 + cos(i * 360 / n / 57.3) * 23, WHITE);
      }

      display.setTextSize(2);

      display.setCursor(80, 20);
      display.print("Ball");
      display.setCursor(80, 50);
      if (bb != 1000)
        sprintf(str, "%d", bb);
      else
        sprintf(str, "Non");
      display.print(str);

      display.setTextSize(1);
      display.setCursor(80, 0);
      //sprintf(str, "%d", ball.distance / 100);
      //display.print(str);
      {
        display.setTextSize(1);
        display.setCursor(80, 0);
        //display.print(roll);
        display.setCursor(95, 0);
        display.print("");
      }
      if (bb != 1000)neo_dir(bb, 255, 0, 0);



    } else if (MODE == 1) {
      if (!SW2) {
        while (!SW2);
        while (SW2) {
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 0);
          display.print(analogRead(Pin_line4));
          display.setCursor(0, 15);
          display.print(analogRead(Pin_line8));
          display.setCursor(0, 30);
          display.print(analogRead(Pin_line12));
          display.setCursor(0, 45);
          display.print(analogRead(Pin_line16));
          display.setCursor(50, 0);
          display.print((analogRead(Pin_line4) > lineF[0]));
          display.setCursor(50, 15);
          display.print((analogRead(Pin_line8) > lineF[1]));
          display.setCursor(50, 30);
          display.print((analogRead(Pin_line12) > lineF[2]));
          display.setCursor(50, 45);
          display.print((analogRead(Pin_line16) > lineF[3]));
          display.display();
          if (!SW3)break;
          if (TS)break;
          if (!SW1) {
            while (!SW1);
            if (!SW3)break;
            if (TS)break;
            tone(buzzer, 2500, 50);
            delay(50);
            clr_timer(30);

            int minl[4] = {1024, 1024, 1024, 1024}, maxl[4] = {0};
            int lval[4];
            while (SW1) {
              if (get_timer(30) > 1000) {
                tone(buzzer, 1600, 200);
                clr_timer(30);
              }
              lval[0] = analogRead(Pin_line4);
              lval[1] = analogRead(Pin_line8);
              lval[2] = analogRead(Pin_line12);
              lval[3] = analogRead(Pin_line16);
              for (int i = 0; i < 4; i++) {
                if (lval[i] > maxl[i])maxl[i] = lval[i];
                if (lval[i] < minl[i])minl[i] = lval[i];
              }
              display.clearDisplay();
              display.setTextSize(2);
              for (int i = 0; i < 4; i++) {
                display.setCursor(0, i * 15);
                display.print(minl[i]);
                display.setCursor(50, i * 15);
                display.print(maxl[i]);
              }
              display.display();
            } while (!SW1);
            tone(buzzer, 2000, 200); delay(200);
            for (int i = 0; i < 4; i++) {
              lineF[i] = ((minl[i] + maxl[i]) / 2);
            }
          }
        } while (!SW2);
        if (1) {
          for (int i = 0; i < 4; i++) {
            EEPROM.write(20 + i, lineF[i]);
          }
          tone(buzzer, 1000, 200);
          delay(500);
          tone(buzzer, 1300, 200);
          delay(500);

        }
        while (!SW1);
      }
      check_line();
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255, 0, 0);
      }
      display.drawCircle(40, 32, 30, WHITE);
      lineUI(37, 29 - 6 * 4, line[0]);
      lineUI(37, 29 - 6 * 3, line[1]);
      lineUI(37, 29 - 6 * 2, line[2]);
      lineUI(37, 29 - 6 * 1, line[3]);

      lineUI(37 + 6 * 4, 29, line[4]);
      lineUI(37 + 6 * 3, 29, line[5]);
      lineUI(37 + 6 * 2, 29, line[6]);
      lineUI(37 + 6 * 1, 29, line[7]);

      lineUI(37, 29 + 6 * 4, line[8]);
      lineUI(37, 29 + 6 * 3, line[9]);
      lineUI(37, 29 + 6 * 2, line[10]);
      lineUI(37, 29 + 6 * 1, line[11]);

      lineUI(37 - 6 * 4, 29, line[12]);
      lineUI(37 - 6 * 3, 29, line[13]);
      lineUI(37 - 6 * 2, 29, line[14]);
      lineUI(37 - 6 * 1, 29, line[15]);


      display.setTextSize(2);

      display.setCursor(80, 20);
      display.print("Line");

      display.setTextSize(1);

      display.setCursor(80, 50);
      display.print(linex);
      display.setCursor(100, 50);
      display.print(liney);

      if (line[1] + line[2] + line[3] + line[4] + line[5] + line[6] + line[7] + line[8] + line[9] + line[10] + line[11] + line[12] + line[13] + line[14] + line[15] + line[0]) {
        //tone(buzzer, 2000);
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(0, 255, 0);
        }
      } else {
        //noTone(buzzer);
      }
      if (linex != 0 || liney != 0) {
        //tone(buzzer, 1600);
      } else {
        //noTone(buzzer);
      }
      /*
        display.setTextSize(2);
        display.setCursor(52, 0);
        sprintf(str, "%d", line[3]);
        display.print(str);

        display.setCursor(80, 22);
        sprintf(str, "%d", line[7]);
        display.print(str);

        display.setCursor(52, 44);
        sprintf(str, "%d", line[11]);
        display.print(str);

        display.setCursor(20, 22);
        sprintf(str, "%d", line[15]);
        display.print(str);
      */

    } else if (MODE == 2) {
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }
      int bb = dir - Dir;
      if (bb < -179)bb += 360;
      if (bb > 180)bb -= 360;
      display.drawCircle(40, 32, 25, WHITE);

      //display.drawLine(40, 32 + 32, 40, 32 - 32, WHITE);
      //display.drawLine(40 - 32, 32, 40 + 32, 32, WHITE);
      /*
        int n = 16;
        for (int i = 0; i < n; i++) {
        display.drawLine(40 + sin(i * 360 / n / 57.3) * 26, 32 + cos(i * 360 / n / 57.3) * 26, 40 + sin(i * 360 / n / 57.3) * 24, 32 + cos(i * 360 / n / 57.3) * 24, WHITE);
        }
      */
      display.setTextSize(2);
      display.drawLine(40 + sin(-bb / 57.3) * 10, 32 - cos(-bb / 57.3) * 10, 40 + sin(-bb / 57.3) * 35, 32 - cos(-bb / 57.3) * 35, WHITE);


      display.setCursor(80, 20);
      display.print("Gyro");
      display.setCursor(80, 50);
      sprintf(str, "%d", bb);
      display.print(str);
      neo_dir(-bb, 255, 0, 0);
    } else if (MODE == 3) {

      /*while (1) {
        display.clearDisplay();
        digitalWrite(Pin_out2, com1 % 2);
        digitalWrite(Pin_out1, com1 % 2);
        com1++;
        checkping();
        digitalWrite(Pin_out4, !SWITCH2);
        digitalWrite(Pin_out3, !SWITCH1);

        display.setTextSize(2);
        display.setCursor(20, 0);
        //display.print("TWELITE");
        checkping();
        display.setTextSize(2);
        display.setCursor(0, 40);
        display.print(digitalRead(Pin_in1));
        display.setCursor(15, 40);
        display.print(digitalRead(Pin_in2));
        display.setCursor(30, 40);
        display.print(digitalRead(Pin_in3));
        display.setCursor(45, 40);
        display.print(digitalRead(Pin_in4));
        checkping();

        if (c2 != digitalRead(Pin_in1)) {
          c2 = !c2;
          nn = 0;
          a++;
        } else {
          nn++;
        }
        if (nn > 20) {
          conect = 0;
          a = 0;
        } else {
          if (a > 5)conect = 1;
        }

        if (conect == 0) {
          display.setCursor(0, 0);
          display.print("Disconnect");
        } else {
          display.setCursor(20, 0);
          display.print("connect");
        }

        if ((conect) == 0) {
          if (get_timer(5) > 1000) {
            tone(buzzer, 1661, 100);
            clr_timer(5);
            ping = get_timer(6);
          }
        } else {
          noTone(buzzer);
        }
        checkping();
        display.setTextSize(2);
        display.setCursor(35, 20);
        display.print(ping);
        int xxxx;
        if (ping >= 10000)xxxx = 70;
        else if (ping >= 1000)xxxx = 60;
        else if (ping >= 100)xxxx = 50;
        else if (ping >= 10)xxxx = 40;
        display.setCursor(xxxx * 1.05 + 20, 20);
        display.setTextSize(2);
        display.print("ms");

        display.setCursor(60, 40);
        display.print(int(ave));

        display.display();
        clr_timer(13);
      */
      get_sensors();
      display.setTextSize(1);
      display.setCursor(0, 0);
      sprintf(str, "A0=%d", ball.value[0]);
      display.print(str);
      display.setCursor(0, 14);
      sprintf(str, "A1=%d", ball.value[1]);
      display.print(str);
      display.setCursor(0, 28);
      sprintf(str, "A2=%d", ball.value[2]);
      display.print(str);
      display.setCursor(0, 42);
      sprintf(str, "A3=%d", ball.value[3]);
      display.print(str);
      display.setCursor(60, 0);
      sprintf(str, "A4=%d", ball.value[4]);
      display.print(str);
      display.setCursor(60, 14);
      sprintf(str, "A5=%d", ball.value[5]);
      display.print(str);
      display.setCursor(60, 28);
      sprintf(str, "A6=%d", ball.value[6]);
      display.print(str);
      display.setCursor(60, 42);
      sprintf(str, "A7=%d", ball.value[7]);
      display.print(str);

      display.setCursor(0, 56);
      sprintf(str, "H1=%d", analogRead(Pin_hold));
      display.print(str);
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setHue(int(((255 / NUM_LEDS * i)) % 255));
      }
    } else if (MODE == 4) {
      //------------------------------------------------------
      //旧カメラチェック
      /*
        for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
        }
        display.setTextSize(2);
        display.setCursor(80, 20);
        display.print("Pixy");

        if (TS1) {
        display.setCursor(80, 40);
        display.print("Y");
        goalcolor = 2;
        seme.color = 2;
        mamori.color = 3;
        } else {
        display.setCursor(80, 40);
        display.print("B");
        goalcolor = 3;
        seme.color = 3;
        mamori.color = 2;
        }
        display.setTextSize(1);

        int k = pixy.ccc.getBlocks();
        int cccc = 0;
        for (int i = 0; i < k; i++) {
        if (pixy.ccc.blocks[i].m_signature == seme.color) {
          seme.y = -(pixy.ccc.blocks[i].m_x - 150);
          seme.x = pixy.ccc.blocks[i].m_y - 120;
          seme.h = (pixy.ccc.blocks[i].m_width);
          seme.w = pixy.ccc.blocks[i].m_height;
          cccc++;
        }
        }
        int cD = degrees(atan2(seme.x, seme.y));
        if (cccc > 0) {
        display.setCursor(0, 0);
        display.print("X=");
        display.setCursor(15, 0);
        display.print(seme.x);
        display.setCursor(0, 10);
        display.print("Y=");
        display.setCursor(15, 10);
        display.print(seme.y);
        display.setCursor(5, 20);
        display.print(cD);
        display.setCursor(0, 30);
        display.print("H=");
        display.setCursor(15, 30);
        display.print(seme.h);
        display.setCursor(0, 40);
        display.print("W=");
        display.setCursor(15, 40);
        display.print(seme.w);
        } else {
        display.setCursor(0, 20);
        display.print("NO");
        cD = 1000;

        }
      */
      //----------------------------------------------
      //新カメラチェック
      display.setTextSize(2);
      display.setCursor(30, 20);
      display.print("Pixy");
      if (!SW2) {
        while (!SW2);
        cameramonitor();
      }


    } else if (MODE == 5) {
      display.setTextSize(2);
      display.setCursor(30, 20);
      display.print("Kicker");
      if (SW2 == 0) {
        while (SW2 == 0);
        kickertest();
      }
    } else if (MODE == 6) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(40, 20);
      display.print("Music");
      display.display();
      if (!SW2) {
        while (!SW2);
        tone(buzzer, 1661, 50);
        delay(50);
        music();
        myDFPlayer.stop();
        while (TS);
      }

    } else {
      positionmonitor();
      /*
        //ラジコン
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(20, 15);
        display.print("RC car");

        if (roll > 10 || roll < -10) {
        m_power[0] = 0;
        m_power[1] = 0;
        m_power[2] = 0;
        m_power[3] = 0;
        Dir = get_dir();
        }
        //display.display();
        int A = 0;
        int D;
        if (conect == 1) {

        if (comin(2)) {
          A += 4;
        }
        if (comin(3)) {
          A += 2;
        }
        if (comin(4)) {
          A += 1;
        }
        if (A == 1) {
          D = 0;
        } else if (A == 2) {
          D = -90;
        } else if (A == 3) {
          D = 90;
        } else if (A == 4) {
          D = 180;
        } else if (A == 5) {
          m_power[0] = -40;
          m_power[1] = 40;
          m_power[2] = 40;
          m_power[3] = -40;
          Dir = get_dir();
        } else if (A == 6) {
          m_power[0] = 40;
          m_power[1] = -40;
          m_power[2] = -40;
          m_power[3] = 40;
          Dir = get_dir();
        } else {
          m_power[0] = 0;
          m_power[1] = 0;
          m_power[2] = 0;
          m_power[3] = 0;
        }
        } else {
        m_power[0] = 0;
        m_power[1] = 0;
        m_power[2] = 0;
        m_power[3] = 0;
        A = -1;
        }
        if (A <= 4 && A>0) {
        if (TS) {
          if (check_line()) {
            cal_line();
            clr_timer(0);
          }
        }
        cal_motorspeed(D,80);
        }
        if(TS){
        kicker();
        }
        Dir %= 360;
        if (A != 5 && A != 6) {
        if (get_timer(11) > 500)
          dircontrol();
        else
          Dir = get_dir();
        } else {
        clr_timer(11);
        }
        pwm_out();

        display.setCursor(30, 35);
        display.print(A);

        //接続チェック
        //--------------------------------------
        if (get_timer(9) > 250) {
        AA = !AA;
        digitalWrite(Pin_out1, AA);
        clr_timer(9);
        }
        if (BB != digitalRead(Pin_in1)) {
        clr_timer(10);
        BB = !BB;
        }
        if (get_timer(10) > 450) {
        conect = 0;
        } else {
        conect = 1;
        }
      */
      //------------------------------------
    }

    display.display();

    if (SW3 == 0) {
      while (SW3 == 0);
      tone(buzzer, 1661, 50);
      MODE++;
      if (MODE > MODE_MAX)MODE = 0;
      else if (MODE < 0)MODE = MODE_MAX;
      delay(50);
    }

    FastLED.show();
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(25, 28);
  display.print("START");
  display.display();
  linex = 0;
  liney = 0;
  clr_timer(24);
  FPS = 0;
}

void positionmonitor() {
  char *pos;
  get_pixy();
  get_sensors();
  check_line();
  if (roll > 10 || roll < -10) {
    linex = 0;
    liney = 0;
  }
  display.drawCircle(40, 32, 25, WHITE);
  if (seme.cansee)
    display.drawLine(40, 32, 40 + seme.x, 32 - seme.y, WHITE);
  if (mamori.cansee)
    display.drawLine(40, 32, 40 + mamori.x, 32 - mamori.y, WHITE);
  pos = "";
  if (liney > 0) {
    if (seme.cansee && seme.dir < 30 && seme.dir > -30) {
      pos = "F Goal";
    } else if ((seme.cansee && seme.dir > 0) || (!seme.cansee && mamori.cansee && mamori.dir > 0)) {
      pos = "LF End";
    } else if ((seme.cansee && seme.dir < 0) || (!seme.cansee && mamori.cansee && mamori.dir < 0)) {
      pos = "RF End";
    } else {
      pos = "unknown";
    }
  } else if (liney < 0) {
    if (mamori.cansee && mamori.dir > 150 || mamori.dir < -150) {
      pos = "B Goal";
    } else if ((mamori.cansee && mamori.dir > 0) || (!mamori.cansee && seme.cansee && seme.dir > 0)) {
      pos = "LB End";
    } else if ((mamori.cansee && mamori.dir < 0) || (!mamori.cansee && seme.cansee && seme.dir < 0)) {
      pos = "RB End";
    } else {
      pos = "unknown";
    }
  } else if (linex != 0) {
    if (seme.cansee && seme.dir > 0 && mamori.cansee && mamori.dir > 0) {
      pos = "L Side";
    } else if (seme.cansee && seme.dir < 0 && mamori.cansee && mamori.dir < 0) {
      pos = "R Side";
    } else if ((!seme.cansee && mamori.cansee && mamori.dir > 0) || (!mamori.cansee && seme.cansee && seme.dir > 0)) {
      pos = "L Side";
    } else if ((!seme.cansee && mamori.cansee && mamori.dir < 0) || (!mamori.cansee && seme.cansee && seme.dir < 0)) {
      pos = "R Side";
    } else {
      pos = "unknown";
    }
  }
  display.setTextSize(1);
  display.setCursor(80, 25);
  display.print(pos);
}


void lineUI(int x, int y, int o) {
  if (o == 1) {
    display.fillRect(x, y, 5, 5, WHITE);
  } else {
    display.drawRect(x + 2, y + 2, 1, 1, WHITE);
  }
}

void kickertest() {
  tone(buzzer, 2000, 50);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print("Kick test");
  display.display();
  while (SW1 == 1) {
    if (SW2 == 0) {
      tone(buzzer, 1500, 50);
      kicker(1);
      delay(100);
      kicker(0);
      delay(300);
    }
  }
  while (SW1 == 0);
  tone(buzzer, 1661, 50);
  delay(50);

}



//サブプログラム
//------------------------------------------------------------------------------------------------------------
void cameramonitor() {
  tone(buzzer, 2000, 50);
  while (SW1) {
    display.clearDisplay();
    display.setTextSize(2);
    if (!SW3) {
      while (!SW3);
      tone(buzzer, 1661, 50);
      if (goalcolor == 2) {
        goalcolor = 3;
        seme.color = 3;
        mamori.color = 2;
      } else {
        goalcolor = 2;
        seme.color = 2;
        mamori.color = 3;
      }
    }//カラー切り替えIF
    get_pixy();
    if (goalcolor == 2) {
      display.setCursor(20, 50);
      display.print("Yellow");
    } else {
      display.setCursor(40, 50);
      display.print("Blue");
    }
    if (seme.cansee)display.drawRect((seme.x / 2) + 64 - seme.w / 4, 50 - (seme.y / 2) - seme.h / 4, seme.w / 2, seme.h / 2, WHITE);
    display.display();
  }
  while (!SW1);
  tone(buzzer, 2000, 50);
  delay(50);
  EEPROM.write(50, seme.color);
}

void cameramonitorold() {
  tone(buzzer, 2000, 50);
  while (SW1) {
    display.clearDisplay();
    display.setTextSize(2);
    if (!SW3) {
      while (!SW3);
      tone(buzzer, 1661, 50);
      if (goalcolor == 2) {
        goalcolor = 3;
        seme.color = 3;
        mamori.color = 2;
      } else {
        goalcolor = 2;
        seme.color = 2;
        mamori.color = 3;
      }
    }//カラー切り替えIF
    get_pixy();
    if (goalcolor == 2) {
      display.setCursor(20, 5);
      display.print("Yellow");
    } else {
      display.setCursor(40, 5);
      display.print("Blue");
    }
    if (seme.cansee == 1) {
      display.setTextSize(1);
      display.setCursor(0, 40);
      display.print("X=");
      display.setCursor(20, 40);
      display.print(seme.x);
      display.setCursor(60, 40);
      display.print("Y=");
      display.setCursor(80, 40);
      display.print(seme.y);
      display.setCursor(50, 55);
      display.print(seme.dir);
    } else {
      display.setCursor(0, 40);
      display.print("Can't see");
    }
    display.display();
  }
  while (!SW1);
  tone(buzzer, 2000, 50);
  delay(50);
  EEPROM.write(50, seme.color);
}


void kicker() {
  if (!IFHOLD) {
    clr_timer(4);
  }

  if (get_timer(4) > 200 && kickerf == 0 && (seme.dir <= 10 && seme.dir >= -10)) {
    digitalWrite(Pin_kicker, HIGH);
    clr_timer(3);
    kickerf = 1;
#ifdef SE_ON
    myDFPlayer.play(9);
#endif
  }

  if (get_timer(3) > 100 && kickerf == 1) {
    kickerf = 0;
    clr_timer(4);
  }
  if (kickerf == 0) {
    digitalWrite(Pin_kicker, LOW);
  }

}


void pwm_out() {
  motor_move(m_power[0], m_power[1], m_power[2], m_power[3]);
}

void get_sensors() {               //センサーの値を取得して変数に入れる.
  line[0] = digitalRead(Pin_line1);
  line[1] = digitalRead(Pin_line2);
  line[2] = digitalRead(Pin_line3);
  line[3] = analogRead(Pin_line4) > lineF[0] ? 1 : 0;
  line[4] = digitalRead(Pin_line5);//
  line[5] = digitalRead(Pin_line6);
  line[6] = digitalRead(Pin_line7);
  line[7] = analogRead(Pin_line8) > lineF[1] ? 1 : 0;
  line[8] = digitalRead(Pin_line9);
  line[9] = digitalRead(Pin_line10);
  line[10] = digitalRead(Pin_line11);
  line[11] = analogRead(Pin_line12) > lineF[2] ? 1 : 0;
  line[12] = digitalRead(Pin_line13);
  line[13] = digitalRead(Pin_line14);
  line[14] = digitalRead(Pin_line15);
  line[15] = analogRead(Pin_line16) > lineF[3] ? 1 : 0;

  if (1) {
    dir = get_dir();
    ball.value[0] = analogRead(Pin_ball1);
    ball.value[1] = analogRead(Pin_ball2);
    ball.value[2] = analogRead(Pin_ball3);
    ball.value[3] = analogRead(Pin_ball4);
    ball.value[4] = analogRead(Pin_ball5);
    ball.value[5] = analogRead(Pin_ball6);
    ball.value[6] = analogRead(Pin_ball7);
    ball.value[7] = analogRead(Pin_ball8);
    clr_timer(37);
    for (int i = 0; i < 8; i++) {
      if (ball.value[i] > 1000)
        ball.value[i] = 1000;
    }
  }
}

int cal_line() {
  int x1 = linex;
  int y1 = liney;
  int d;
  int r = 3;
  int rr = 1;;
  if (seme.cansee && (seme.dir < 50 && seme.dir > -50) && mamori.cansee && (mamori.dir > 130 || mamori.dir < -130)) {
    //r=8;
    //rr=6;
  }

  if (x1 >= r) {
    if (y1 >= r) {
      d = -135;
    } else if (y1 <= -r) {
      d = -45;
    } else {
      d = -90;
    }
  } else if (x1 <= -r) {
    if (y1 >= 4) {
      d = 135;
    } else if (y1 <= -r) {
      d = 45;
    } else {
      d = 90;
    }
  } else {
    if (y1 >= r) {
      d = 180;
    } else if (y1 <= -r) {
      d = 0;
    } else {


      if (x1 >= rr) {
        if (ver39 == 1)Speed = 60;
        if (0) {
          if (dir_move > 0 && dir_move < 180) {
            if (ball.dir < 110) {
              dir_move = 0;
            } else if (ball.dir > 135) {
              dir_move = 180;
            } else {
              dir_move = 1000;
            }
          }
        } else {
          if (dir_move > 0 && dir_move < 180) {
            if (dir_move < 60) {
              dir_move = 0;
            } else if (dir_move > 90) {
              dir_move = 180;
            } else {
              dir_move = 1000;
            }
          }
        }
      } else if (x1 <= -rr) {
        if (ver39 == 1)Speed = 60;
        if (0) {
          if (dir_move < 0 && dir_move > -180) {
            if (ball.dir > -110) {
              dir_move = 0;
            } else if (ball.dir < -135) {
              dir_move = 180;
            } else {
              dir_move = 1000;
            }
          }
        } else {
          if (dir_move < 0 && dir_move > -180) {
            if (dir_move > -60) {
              dir_move = 0;
            } else if (dir_move < -90) {
              dir_move = 180;
            } else {
              dir_move = 1000;
            }
          }
        }
      }
      if (dir_move == 1000) {
        return 0;
      }

      if (y1 >= rr) {
        if (ver39 == 1)Speed = 60;
        if (dir_move > -90 && dir_move < 90) {
          if (dir_move > 30) {
            dir_move = 90;
          } else if (dir_move < -30) {
            dir_move = -90;
          } else {
            dir_move = 1000;
          }
        }
      } else if (y1 <= -rr) {
        if (ver39 == 1)Speed = 60;
        if (dir_move > 90 || dir_move < -90) {
          if (ball.dir > 0) {
            dir_move = 90;
          } else if (ball.dir < 0) {
            dir_move = -90;
          } else {
            dir_move = 1000;
          }
        }
      }
      //Speed = MAXspeed;
      return 0;
    }
  }
  //neo_dir(d,255,0,0);
  //LINEd = d;
  if (linex > 4 || linex < -4 || liney > 4 || liney < -4) {

  } else {
    Speed = 80;
  }
  dir_move = d;
  return 0;

}

int check_line() {
  int li[4];
  int xx, yy;
  bool L = 0;
  for (int i = 0; i < 4; i++) {
    if (line[4 * i + 3]) {
      li[i] = 4;
    } else if (line[4 * i + 2]) {
      li[i] = 3;
    } else if (line[4 * i + 1]) {
      li[i] = 2;
    } else if (line[4 * i]) {
      li[i] = 1;
    } else {
      li[i] = 0;
    }
  }

  if (liney == 0 && li[0] > 2 && li[2] == 0) {
    yy = 2;
  } else if (liney == 0 && li[0] == 0 && li[2] > 2) {
    yy = -2;
  } else {
    if (li[0] > 0 && li[2] == 0) {
      yy = li[0]; L = 1;
    } else if (li[0] == 0 && li[2] > 0) {
      yy = -li[2]; L = 1;
    } else if (li[0] > 0 && li[2] > 0) {
      yy = 0; L = 1;
    } else {
      yy = 0;
    }
  }
  if (linex == 0 && li[1] > 2 && li[3] == 0) {
    xx = 2;
  } else if (linex == 0 && li[1] == 0 && li[3] > 2) {
    xx = -2;
  } else {
    if (li[1] > 0 && li[3] == 0) {
      xx = li[1]; L = 1;
    } else if (li[1] == 0 && li[3] > 0) {
      xx = -li[3]; L = 1;
    } else if (li[1] > 0 && li[3] > 0) {
      xx = 0; L = 1;
    } else {
      xx = 0;
    }
  }

  if (L >= 0) {
    int aa = 2;

    if (linex > aa && xx < -aa) {
      xx = 9 + xx;
    }
    if (linex < -aa && xx > aa) {
      xx = -9 + xx;
    }
    if (liney > aa && yy < -aa) {
      yy = 9 + yy;
    }
    if (liney < -aa && yy > aa) {
      yy = -9 + yy;
    }
    if (linex >= 5 && xx < 0) {
      xx = 9 + xx;
    }
    if (linex <= -5 && xx > 0) {
      xx = -9 + xx;
    }
    if (liney >= 5 && yy < 0) {
      yy = 9 + yy;
    }
    if (liney <= -5 && yy > 0) {
      yy = -9 + yy;
    }
    if (xx == 0 && (linex > 2 || linex < -2)) {
      xx = linex;
    }
    if (yy == 0 && (liney > 2 || liney < -2)) {
      yy = liney;
    }
    if (linex >= 3 || linex <= -3) {
      if (liney == 0 && liney == 0)yy = 0;
    }
    if (liney >= 3 || liney <= -3) {
      if (linex == 0 && linex == 0)xx = 0;
    }
    linex = xx;
    liney = yy;
  }
  if (xx == 0 && yy == 0 && linex * linex <= 16 && liney * liney <= 16) {
    linex = 0;
    liney = 0;
  }
  if ((line[5] > 0 && line[6] > 0) && (line[9] > 0 || line[10] > 0)) { //角判定
    corner = 1;
    if (kadomode == 0)kadomode = 2;
    clr_timer(11);
    clr_timer(12);
  }
  if ((line[13] > 0 && line[14] > 0) && (line[9] > 0 || line[10] > 0)) {
    corner = 1;
    if (kadomode == 0)kadomode = -2;
    clr_timer(11);
    clr_timer(12);
  }
  if (linex > 0 && (mamori.dir > 90) * 0) {
    if (kadomode == 0)kadomode = 2;
    clr_timer(12);
  }
  if (linex < 0 && (mamori.dir < -90) * 0) {
    if (kadomode == 0)kadomode = -2;
    clr_timer(12);
  }

  if (kadomode == 2 || kadomode == -2) {
    if (get_timer(12) > 300) {
      kadomode /= 2;
    }
  }
  if (get_timer(12) > 700) {
    kadomode = 0;
  }

  if (get_timer(11) > 1500) {
    corner = 0;
  }

  if (linex != 0 || liney != 0) {
    return 1;
  }
  return 0;
}

int check_lineold() {
  int x = linex, y = liney; //書きやすいように
  int d;

  if (1) {
    if (line[0] == 1) {
      y = 1;
    }
    if (line[1] == 1) {
      x = 1;
    }
    if (line[2] == 1) {
      y = -1;
    }
    if (line[3] == 1) {
      x = -1;
    }
    if (line[4] == 1) {
      y = 2;
    }
    if (line[5] == 1) {
      x = 2;
    }
    if (line[6] == 1) {
      y = -2;
    }
    if (line[7] == 1) {
      x = -2;
    }
  }



  if (x >= 2 ) {
    if (y >= 2) {
      d = -135;
    } else if (y <= -2) {
      d = -45;
    } else {
      d = -90;
    }
  } else if (x <= -2) {
    if (y >= 2) {
      d = 135;
    } else if (y <= -2) {
      d = 45;
    } else {
      d = 90;
    }
  } else {
    if (y >= 2) {
      d = 180;
    } else if (y <= -2) {
      d = 0;
    } else {
      if (x >= 1) {
        if (dir_move > 0 && dir_move < 180) {
          if (dir_move < 60) {
            dir_move = 0;
          } else if (dir_move > 120) {
            dir_move = 180;
          } else {
            dir_move = 1000;
          }
        }
      } else if (x <= -1) {
        if (dir_move < 0 && dir_move > -180) {
          if (dir_move > -60) {
            dir_move = 0;
          } else if (dir_move < -120) {
            dir_move = 180;
          } else {
            dir_move = 1000;
          }
        }
      }
      if (dir_move == 1000) {
        return 0;
      }

      if (y >= 1) {
        if (dir_move > -90 && dir_move < 90) {
          if (dir_move > 30) {
            dir_move = 90;
          } else if (dir_move < -30) {
            dir_move = -90;
          } else {
            dir_move = 1000;
          }
        }
      } else if (y <= -1) {
        if (dir_move > 90 || dir_move < -90) {
          if (dir_move > 150) {
            dir_move = 90;
          } else if (dir_move < -150) {
            dir_move = -90;
          } else {
            dir_move = 1000;
          }
        }
      }

      return 0;
    }
  }
  //neo_dir(d,255,0,0);
  LINEd = d;
  return 1;
}



int check_line_old() {
  int x = linex, y = liney; //書きやすいように
  int d;
  x = 0; y = 0;
  if (1) {
    if (line[0] + line[1] + line[2] + line[3]) {
      y += 1;
    }
    if (line[4] + line[5] + line[6] + line[7]) {
      x += 1;
    }
    if (line[8] + line[9] + line[10] + line[11]) {
      y += -1;
    }
    if (line[12] + line[13] + line[14] + line[15]) {
      x += -1;
    }

    if (line[2] + line[3]) {
      y += 1;
    }
    if (line[6] + line[7]) {
      x += 1;
    }
    if (line[10] + line[11]) {
      y += -1;
    }
    if (line[14] + line[15]) {
      x += -1;
    }

  }


  if (lx * x < 0)
    x = lx;
  if (ly * y < 0)
    y = ly;



  if (x > 0) {
    if (y > 0) {
      d = -135;
    } else if (y < 0) {
      d = -45;
    } else {
      d = -90;
    }
  } else if (x < 0) {
    if (y > 0) {
      d = 135;
    } else if (y < 0) {
      d = 45;
    } else {
      d = 90;
    }
  } else {
    if (y > 0) {
      d = 180;
    } else if (y < 0) {
      d = 0;
    } else {
      lx = 0;
      ly = 0;
      return 0;
    }
  }
  if (x != 0)
    lx = x;
  if (y != 0)
    ly = y;

  if (x > 1 || x < -1 || y > 1 || y < -1) {
    LINEd = d;
  } else {
    int xx, yy;
    xx = sin(dir_move / 57.3) * 100;
    yy = cos(dir_move / 57.3) * 100;
    if (x > 0 && xx > 0)
      xx = 0;
    else if (x < 0 && xx < 0)
      xx = 0;
    if (y > 0 && yy > 0)
      yy = 0;
    else if (y < 0 && yy < 0)
      yy = 0;
    if ((xx * xx + yy * yy) > 10000 * 0.3)
      LINEd = atan2(xx, yy) * 57.3;
    else
      LINEd = 1000;
  }

  return 1;
}

void neo_dir(int dir, int R, int G, int B) {
  if (dir == 1000)return;
  dir %= 360;
  if (dir < 0)dir += 360;
  int n = map(dir, 0, 360, 0, 17);
  leds[(n + 15) % 16] = CRGB(R, G, B);
  leds[(n + 0) % 16] = CRGB(R, G, B);
  leds[(n + 1) % 16] = CRGB(R, G, B);
}

int get_ball()
{
  ball.distance = 0;
  for (int i = 0; i < 8; i++) {
    ball.n = 0;
    ball.distance += ball.value[i];
    if (ball.value[i] < 1000) {
      ball.n++;
    }
  }
  int r = 1000;
  if (Robotn == 1 && conect) {
    r = 600;
  }
  for (int i = 0; i < 8; i++) {
    if (ball.value[i] < r)break;

    if (i == 7)return 1000;
  }

  int x = 0, y = 0;
  x += ball.value[6];
  x -= ball.value[2];
  x -= ((ball.value[1] + ball.value[3] - ball.value[5] - ball.value[7]) * 0.7071);

  y += ball.value[4];
  y -= ball.value[0];
  y -= ((ball.value[1] + ball.value[7] - ball.value[5] - ball.value[3]) * 0.7071);

  return atan2(x, y) * 57;
}





void motor_move2(int p1, int p2, int p3, int p4) {
  double m1 = p1, m2 = p2, m3 = p3, m4 = p4;
  if (p1 == 1000) {
    motor[0] = 0;
    motor[1] = 0;
  } else {
    if (m1 > 0) {
      motor[0] = double(m1 / 100);
      motor[1] = 0;
    } else if (m1 < 0) {
      motor[0] = 0;
      motor[1] = double(-m1 / 100);
    } else {
      motor[0] = 1;
      motor[1] = 1;

    }
  }
  if (p2 == 1000) {
    motor[2] = 0;
    motor[3] = 0;
  } else {
    if (m2 > 0) {
      motor[2] = double(m2 / 100);
      motor[3] = 0;
    } else if (m2 < 0) {
      motor[2] = 0;
      motor[3] = double(-m2 / 100);
    } else {
      motor[2] = 1;
      motor[3] = 1;

    }
  }
  if (p3 == 1000) {
    motor[4] = 0;
    motor[5] = 0;
  } else {
    if (m3 > 0) {
      motor[4] = double(m3 / 100);
      motor[5] = 0;
    } else if (m3 < 0) {
      motor[4] = 0;
      motor[5] = double(-m3 / 100);
    } else {
      motor[4] = 1;
      motor[5] = 1;
    }
  }
  if (p4 == 1000) {
    motor[6] = 0;
    motor[7] = 0;
  } else {
    if (m4 > 0) {
      motor[6] = double(m4 / 100);
      motor[7] = 0;
    } else if (m4 < 0) {
      motor[6] = 0;
      motor[7] = double(-m4 / 100);
    } else {
      motor[6] = 1;
      motor[7] = 1;

    }
  }

}

void motor_move(int p1, int p2, int p3, int p4) {
  double m1 = p1, m2 = p2, m3 = p3, m4 = p4;
  if (m1 > 0) {
    motor[0] = 1;
    motor[1] = double((100 - m1) / 100);
  } else if (m1 < 0) {
    motor[0] = double((100 + m1) / 100);
    motor[1] = 1;
  } else {
    motor[0] = 1;
    motor[1] = 1;
  }
  if (m2 > 0) {
    motor[2] = 1;
    motor[3] = double((100 - m2) / 100);
  } else if (m2 < 0) {
    motor[2] = double((100 + m2) / 100);
    motor[3] = 1;
  } else {
    motor[2] = 1;
    motor[3] = 1;
  }
  if (m3 > 0) {
    motor[4] = 1;
    motor[5] = double((100 - m3) / 100);
  } else if (m3 < 0) {
    motor[4] = double((100 + m3) / 100);
    motor[5] = 1;
  } else {
    motor[4] = 1;
    motor[5] = 1;
  }
  if (m4 > 0) {
    motor[6] = 1;
    motor[7] = double((100 - m4) / 100);
  } else if (m4 < 0) {
    motor[6] = double((100 + m4) / 100);
    motor[7] = 1;
  } else {
    motor[6] = 1;
    motor[7] = 1;
  }
}

int mawarikomi() {
  if (ball.dir == 1000)return 1000;
  if (ball.dir < 3 && ball.dir > -3) {
    return 0;
  } else if (ball.dir < 10 && ball.dir > -10) {
    return ball.dir * 1.1;
  } else if (ball.dir < 25 && ball.dir > -25) {
    return ball.dir * 2;
  } else {
    int pp = 50;
    if (ball.dir < 30 && ball.dir > -30) {
      pp = 30;
    } else if (ball.dir < 50 && ball.dir > -50) {
      pp = 40;
    }
    if (ball.distance < 5000) {
      pp *= 1.25;
    }
    if (ball.dir > 0)
      return ball.dir + pp;
    else
      return ball.dir - pp;
    //return ball.dir * 1.5;
  }
}

void dircontrol() {
  int G;
  int sub = dir - Dir;
  if (sub < -179)sub += 360;
  else if (sub > 180)sub -= 360;

  G = sub * 1 - (bd - sub) * 0;
  bd = sub;


  //neo_dir(-sub, 255, 0, 0);
  m_power[0] -= G;
  m_power[1] += G;
  m_power[2] += G;
  m_power[3] -= G;
}

void dircontrol2() {
  int G;
  int sub = dir - Dir;
  if (sub < -179)sub += 360;
  else if (sub > 180)sub -= 360;

  G = sub / 2 - (bd - sub) * 0;
  bd = sub;


  //neo_dir(-sub, 255, 0, 0);
  m_power[0] -= G;
  m_power[1] += G;
  m_power[2] += G;
  m_power[3] -= G;
}

void cal_motorspeed(int D, int Speed) {
  int MAX = 0;
  if (D == 1000) {
    m_power[0] = 0, m_power[1] = 0, m_power[2] = 0, m_power[3] = 0; //停止
  } else {
    m_power[0] = sin((D + 45) / 57.3) * Speed;
    m_power[1] = -sin((D - 45) / 57.3) * Speed;
    m_power[2] = -sin((D - 135) / 57.3) * Speed;
    m_power[3] = sin((D + 135) / 57.3) * Speed;
  }

  for (int i = 0; i < 4; i++) {
    if (abs(m_power[i]) > MAX) {
      MAX = abs(m_power[i]);
    }
  }


  for (int i = 0; i < 4; i++) {
    m_power[i] = m_power[i] * Speed / MAX;
    constrain(m_power[i], -100, 100);
  }
}





//関数
//--------------------------------------------------------------------------------------------------------------
unsigned long get_timer(int n) {
  return millis() - S_timer[n];
}

void clr_timer(int n) {
  S_timer[n] = millis();
}


void get_pixy() {
  int k = pixy.ccc.getBlocks();
  int cc = 0;
  int x, y, d;
  seme.cansee = 1;
  for (int i = 0; i < k; i++) {
    if (pixy.ccc.blocks[i].m_signature == seme.color) {
      y = -(pixy.ccc.blocks[i].m_x - 150);
      x = pixy.ccc.blocks[i].m_y - 120;
      d = degrees(atan2(x, y));
      if (d < 90 && d > -90) {
        seme.x = x;
        seme.y = y;
        seme.dir = d;
        seme.w = pixy.ccc.blocks[i].m_height;
        seme.h = pixy.ccc.blocks[i].m_width;
      }
      cc++;
    }
  }

  if (cc == 0) {
    seme.dir = 0;
    seme.cansee = 0;
  }

  k = pixy.ccc.getBlocks();
  cc = 0;
  mamori.cansee = 1;
  for (int i = 0; i < k; i++) {
    if (pixy.ccc.blocks[i].m_signature == mamori.color) {
      y = -(pixy.ccc.blocks[i].m_x - 150);
      x = pixy.ccc.blocks[i].m_y - 120;
      d = degrees(atan2(x, y));
      if (d > 90 || d < -90) {
        mamori.x = x;
        mamori.y = y;
        mamori.dir = d;
      }
      cc++;
    }
  }
  mamori.dir = degrees(atan2(mamori.x, mamori.y));
  if (cc == 0) {
    mamori.dir = 0;
    mamori.cansee = 0;
  }
}











//初期設定
//--------------------------------------------------------------------------------------------------------------


void neopixel_setup() {
  FastLED.addLeds<NEOPIXEL, neopixel_pin>(leds, NUM_LEDS); //Neopixel初期化処理
  FastLED.setBrightness(BRIGHTNESS);
  pinMode(neopixel_pin, OUTPUT);
}


void bno_setup() {
  bno.begin();
  bno.getTemp();
  bno.setExtCrystalUse(true);
}

int get_dir() {  //方位を求める
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int d = euler.x();
  roll = euler.y();
  if (d > 180)d -= 360;
  return d;
}

//dinogame


#pragma once


const unsigned char bitmap_dino[] PROGMEM = {
  B00000000, B00000001, B11111111, B11000000,
  B00000000, B00000011, B11111111, B11000000,
  B00000000, B00000011, B10111111, B11100000,
  B00000000, B00000011, B10111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111011, B11100000,
  B00000000, B00000011, B11111000, B00000000,
  B00000000, B00000011, B11111111, B10000000,
  B00000000, B00000011, B11100000, B00000000,
  B10000000, B00000111, B11100000, B00000000,
  B10000000, B00011111, B11100000, B00000000,
  B11100000, B01111111, B11111100, B00000000,
  B11100000, B11111111, B11111100, B00000000,
  B11110001, B11111111, B11100100, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B01111111, B11111111, B11100000, B00000000,
  B00011111, B11111111, B11000000, B00000000,
  B00011111, B11111111, B11000000, B00000000,
  B00001111, B11111111, B10000000, B00000000,
  B00000011, B11111110, B00000000, B00000000,
  B00000001, B11100011, B10000000, B00000000,
  B00000001, B11100011, B10000000, B00000000,
  B00000001, B11000000, B00000000, B00000000,
  B00000001, B00000000, B00000000, B00000000,
  B00000001, B10000000, B00000000, B00000000,
  B00000001, B11000000, B00000000, B00000000
};

/**
   Made with Marlin Bitmap Converter
   https://marlinfw.org/tools/u8glib/converter.html

   This bitmap from the file 'dinomini2.png'
*/
/**
   Made with Marlin Bitmap Converter
   https://marlinfw.org/tools/u8glib/converter.html

   This bitmap from the file 'dinomini2.png'
*/
#pragma once

const unsigned char bitmap_dino2[] PROGMEM = {
  B00000000, B00000001, B11111111, B11000000,
  B00000000, B00000011, B11111111, B11000000,
  B00000000, B00000011, B10111111, B11100000,
  B00000000, B00000011, B10111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111111, B11100000,
  B00000000, B00000011, B11111011, B11100000,
  B00000000, B00000011, B11111000, B00000000,
  B00000000, B00000011, B11111111, B10000000,
  B00000000, B00000011, B11100000, B00000000,
  B10000000, B00000111, B11100000, B00000000,
  B10000000, B00011111, B11100000, B00000000,
  B11100000, B01111111, B11111100, B00000000,
  B11100000, B11111111, B11111100, B00000000,
  B11110001, B11111111, B11100100, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B11111111, B11111111, B11100000, B00000000,
  B01111111, B11111111, B11100000, B00000000,
  B00011111, B11111111, B11000000, B00000000,
  B00011111, B11111111, B11000000, B00000000,
  B00001111, B11111111, B00000000, B00000000,
  B00000011, B11111110, B00000000, B00000000,
  B00000001, B11001110, B00000000, B00000000,
  B00000001, B10000110, B00000000, B00000000,
  B00000000, B11100010, B00000000, B00000000,
  B00000000, B00000010, B00000000, B00000000,
  B00000000, B00000010, B00000000, B00000000,
  B00000000, B00000011, B00000000, B00000000
};




#pragma once

const unsigned char bitmap_sabo[] PROGMEM = {
  B00001100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B10000000,
  B11011100, B11000000,
  B11011100, B11000000,
  B11011100, B11000000,
  B11011100, B11000000,
  B11011100, B11000000,
  B11001100, B11000000,
  B11111111, B10000000,
  B01111111, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000,
  B00011100, B00000000
};




/////////////////////////////////////////////////////////////////////////////////////////////

#define MAXSPEED 13
#define JUMPpower 7

double y;
int jump = 0;
int a = 0;

int oy[20];
int ox[20];
int n = 0;

int wx[10];
int wn = 0;

int co = 0;
int score;
int gamespeed = 6;
int hi;
int dinocos = 0;

void reset() {
  randomSeed(analogRead(A9));
  co = 0;
  n = 0;
  y = 34;
  wn = 0;
  a = 0;
  jump = 0;
  y = 34;
  score = 0;
  gamespeed = 6;
}

void gameover() {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(5, 15);
  display.print("GAME OVER");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(65, 0);
  display.print("score:");
  display.setCursor(100, 0);
  display.print(score);
  display.display();

  display.setCursor(0, 0);
  display.print("HI:");
  display.setCursor(20, 0);
  hi = EEPROM.read(20) * 256 + EEPROM.read(21);
  if (score > hi) {
    hi = score;
    EEPROM.write(20, int(score / 256));
    EEPROM.write(21, int(score % 256));
  }
  display.print(hi);
  display.display();
  tone(buzzer, 255, 50);
  delay(100);
  tone(buzzer, 255, 50);
  delay(50);
  while (SW1 && SW2 && SW3);
  if (!SW1) {
    while (!SW1);
    standby();
  }
  delay(200);
  if (SWITCH2 == 0 && SWITCH1 == 0) {
    clr_timer(20);
    while (get_timer(20) <= 3000 || !(SWITCH2 == 0 && SWITCH1 == 0)) {
      ;
    }
    if (get_timer(20) > 3000) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(5, 15);
      display.print("PASSWARD");
      display.display();
      while (SWITCH2 == 0 || SWITCH1 == 0);
      delay(100);
      while (SWITCH2 == 1 && SWITCH1 == 1);
      tone(buzzer, 1600, 50);
      if (SWITCH2 == 0) {
        while (SWITCH2 == 0 || SWITCH1 == 0);
        delay(100);
        while (SWITCH2 == 1 && SWITCH1 == 1);
        tone(buzzer, 1600, 50);
        if (SWITCH2 == 0) {
          while (SWITCH2 == 0 || SWITCH1 == 0);
          delay(100);
          while (SWITCH2 == 1 && SWITCH1 == 1);
          tone(buzzer, 1600, 50);
          if (SWITCH1 == 0) {
            while (SWITCH2 == 0 || SWITCH1 == 0);
            delay(100);
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(5, 15);
            display.print("CLEAR DATA");
            display.setCursor(5, 40);
            display.print("NO");
            display.setCursor(85, 40);
            display.print("YES");
            display.display();
            while (SWITCH2 == 1 && SWITCH1 == 1);
            if (SWITCH2 == 0) {
              EEPROM.write(20, 0);
              EEPROM.write(21, 0);
              tone(buzzer, 2000, 100);
              display.clearDisplay();
              display.setTextSize(2);
              display.setTextColor(WHITE);
              display.setCursor(5, 15);
              display.print("CLEAR DATA");
              display.setCursor(20, 45);
              display.print("FINISH");
              display.display();
              delay(500);

            }
          }
        }
      }
    }
  }
  reset();
}


void dinogame()
{
  reset();
  while (1) {
    int nn;
    int rr;
    if ((!SW2  || !SW3) && jump == 0) {
      a = -JUMPpower;
      jump = 1;
      tone(buzzer, 600, 50);
    }

    y += a;
    display.clearDisplay();
    if (dinocos % 4 < 2) {
      display.drawBitmap(0, y, bitmap_dino, 27, 30, WHITE);
    } else {
      display.drawBitmap(0, y, bitmap_dino2, 27, 30, WHITE);
    }
    if (y == 34)dinocos++;
    display.drawLine(0, 62, 128, 62, WHITE);

    int l = random(0, 3);
    if (l == 0) {
      ox[n] = 128;
      oy[n] = random(62, 66);
      n++;
    }
    if (score > 2000) {
      nn = 18;
    }
    else if (score > 1000) {
      nn = 22;
    } else {
      nn = 25;
    }
    if (score % 500 == 0 && gamespeed < MAXSPEED)
      gamespeed++;

    if (score > 2000) {
      rr = 40;
    } else if (score > 1500) {
      rr = 30;
    } else if (score > 500) {
      rr = 35;
    } else {
      rr = 40;
    }
    int r = random(0, rr - co);
    if (r == 0 && co > nn) {
      wx[wn] = 128;
      wn++;
      co = 0;
      r = random(0, 3);
      if (r == 0) {
        wx[wn] = 138;
        wn++;
      }
      r = random(0, 4);
      if (r == 0 && score > 250) {
        wx[wn] = 148;
        wn++;
      }
      r = random(0, 5);
      if (r == 0 && score > 750) {
        wx[wn] = 158;
        wn++;
      }
      r = random(0, 3);
      if (r == 0 && score > 2500) {
        wx[wn] = 168;
        wn++;
      }
    }


    co++;

    for (int i = 0; i < n; i++) {
      ox[i] -= gamespeed;
      display.fillRect(ox[i], oy[i], 3, 1, WHITE);

    }

    if (ox[0] <= 0 && n > 0) {
      for (int i = 1; i < n; i++) {
        ox[i - 1] = ox[i];
        oy[i - 1] = oy[i];
      }
      n--;
    }

    for (int i = 0; i < wn; i++) {
      //display.fillRect(wx[i],62-20,10,20,WHITE);
      wx[i] -= gamespeed;
      display.drawBitmap(wx[i], 62 - 20, bitmap_sabo, 10, 20, WHITE);

    }

    if (wx[0] <= -5 && wn > 0) {
      for (int i = 1; i < wn; i++) {
        wx[i - 1] = wx[i];
      }
      wn--;
    }


    if (wn != 0 && wx[0] < 20 && y > 15) {
      gameover();
    }
    if (wn > 1 && wx[1] < 20 && y > 15) {
      gameover();
    }
    if (wn > 2 && wx[2] < 20 && y > 15) {
      gameover();
    }
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(65, 0);
    display.print("score:");
    display.setCursor(100, 0);
    display.print(score);

    display.display();
    //delay(1);

    if (a == JUMPpower) {
      a = 0;
      jump = 0;
    }
    if (jump != 0) {
      a++;
    }

    score += 1;


  }
}











//音楽

#define musicMAX 6
void music() {

  int n = 1;
  int s = -1;
  clr_timer(23);
  myDFPlayer.volume(VOLUME);
  while (!TS) {
    if (!SW3) {
      clr_timer(20);
      while (!SW3 && get_timer(20) < 700);
      if (get_timer(20) < 700) {
        tone(buzzer, 2000, 50);
        delay(50);
        n++;
        s = 1;
        if (n > musicMAX)n = 1;
        myDFPlayer.play(n);
        clr_timer(23);
      } else {
        set_volume();
        myDFPlayer.volume(VOLUME);
      }
    }
    if (!SW1) {
      clr_timer(20);
      while (!SW1 && get_timer(20) < 700);
      if (get_timer(20) < 700) {
        tone(buzzer, 2000, 50);
        delay(50);
        n--;
        s = 1;
        if (n < 1)n = musicMAX;
        myDFPlayer.play(n);
        clr_timer(23);
      } else {
        set_volume();
        myDFPlayer.volume(VOLUME);
      }
    }
    if (s == 1) {
      if (get_timer(23) > 400  && digitalRead(Pin_BUSY) == 1)
        s = -1;  //音楽終了
    } else {
      clr_timer(23);
    }
    if (!SW2) {
      while (!SW2);
      if (s == 1) {
        myDFPlayer.pause();
        s = 0;
        clr_timer(23);
      } else {
        if (s == 0) {
          myDFPlayer.start();
          clr_timer(23);
        } else {
          myDFPlayer.play(n);
          clr_timer(23);
        }
        s = 1;
      }
    }
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.print("Music");
    display.setCursor(100, 10);
    display.print(n);
    display.setTextSize(1);
    if (s == 1) {
      display.drawRect(58, 44, 5, 10, WHITE);
      display.drawRect(66, 44, 5, 10, WHITE);
    } else {
      display.drawTriangle(58, 43, 58, 55, 70, 49, WHITE);
    }
    display.drawTriangle(100, 43, 100, 55, 108, 49, WHITE);
    display.drawRect(108, 43, 4, 12, WHITE);
    display.drawTriangle(28, 43, 28, 55, 20, 49, WHITE);
    display.drawRect(16, 43, 4, 12, WHITE);
    display.display();


  }
  tone(buzzer, 1661, 50);
  delay(50);
}


void set_volume() {
  int nowv = VOLUME;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(40, 10);
  display.print("VOLUME");
  display.setCursor(50, 45);
  display.print(VOLUME);
  display.display();

  clr_timer(21); //音量
  clr_timer(22);
  while (get_timer(22) < 1000) {
    if (get_timer(22) > 200 && nowv != VOLUME) {
      myDFPlayer.volume(VOLUME);
      nowv = VOLUME;
    }
    if (!SW3 || !SW1)
      clr_timer(22);
    if (get_timer(21) > 150) {
      if (!SW3 || !SW1) {
        clr_timer(22);
        if (!SW3 && VOLUME < 30) {
          clr_timer(21);
          VOLUME++;
        } else if (!SW1 && VOLUME > 0) {
          clr_timer(21);
          VOLUME--;
        }
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(30, 10);
        display.print("VOLUME");
        display.setCursor(50, 45);
        display.print(VOLUME);
        display.display();
      }
    }
  }
}


void loop() {
  if (get_timer(24) > 1000) {
    display.clearDisplay();
    display.setCursor(30, 20);
    display.setTextSize(4);
    display.print(FPS);
    display.display();
    FPS = 0;
    clr_timer(24);
  }
  FPS++;

  if (digitalRead(Pin_BUSY) == 1) {
    //myDFPlayer.play(5);
  }

  int balljibun;
  if (ball.n <= 2) {
    comout(2, 1);
    comout(3, 0);
    balljibun = 1;
    if (ball.dir == 1000)balljibun = 0;
  } else if (ball.n <= 4) {
    comout(2, 0);
    comout(3, 1);
    balljibun = 2;
  } else {
    comout(2, 1);
    comout(3, 1);
    balljibun = 3;
  }

  Speed = MAXspeed;
  if (!TS) {
    standby();
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  if (conect == 1) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 255, 255);
    }
  }

  get_sensors();
  ball.dir = get_ball();
  neo_dir(ball.dir, 255, 0, 0);

  dir_move = mawarikomi();

  if (ball.dir == 1000 && mamori.w < 120) {
    //dir_move = mamori.dir;
  }

  //------------------------------------------
  //キーパー
  /*
    if(!SW1){
    ifkeeper=2;
    }

    if (ifkeeper == 2) {
    if (liney < 0)
      ifkeeper = 1;
    dir_move = mamori.dir;
    }else if(ifkeeper==1){
    if(ball.value[0]<200)ifkeeper=0;
    if(mamori.dir>150 && mamori.y>-100){
      dir_move=45;
    }else if(mamori.dir<-150 && mamori.y>-100){
      dir_move=-45;
    }else{
      if(ball.dir>30 && ball.dir!=1000){
        if(liney < 0)dir_move=90;
        else dir_move=120;
        if(mamori.x<-50)dir_move=1000;
      }else if(ball.dir<-30){
        if(liney < 0)dir_move=-90;
        else dir_move=-120;
        if(mamori.x>50)dir_move=1000;
      }else{
        dir_move=1000;
      }
    }
    }
  */
  if (mamorimode) {
    if (ball.value[0] < 300 && ball.dir < 10 && ball.dir > -10 && conect && !comin(3)) {
      comout(2, 1);
      tone(buzzer, 2000, 100);
    } else
      comout(2, 0);

    //comout(3, ifkeeper);

    if (comin(2) && !ifkeeper && conect) {
      ifkeeper = 1;
      tone(buzzer, 2000, 100);
    }
    if (ifkeeper == 1) {
      if (liney < 0)
        ifkeeper = 0;
      dir_move = mamori.dir;
    }
  }

  int rr = 0;
  if (comin(2)) {
    rr++;
  } else if (comin(3)) {
    rr += 2;
  }

#ifdef SE_ON
  if (linex == 0 && liney == 0) {
    clr_timer(47);
  } else if (abs(linex) <= 2 && abs(liney) <= 2) {
    if (get_timer(47) > 200) {
      clr_timer(48);
    }
  } else {
    if (get_timer(48) > 2000) {
      myDFPlayer.play(11);
    }
    clr_timer(48);
  }
#endif

  /*if (balljibun > rr && conect && Robotn == 2) { //(ball.dir == 1000 && liney >= 0 && keeperon && Robotn==1) {
    dir_move = mamori.dir;
    if (!mamori.cansee)dir_move = 180;
    Speed = 80;
    }

    if (balljibun >= rr && conect && Robotn == 1) { //(ball.dir == 1000 && liney >= 0 && keeperon && Robotn==1) {
    dir_move = mamori.dir;
    if (!mamori.cansee)dir_move = 180;
    Speed = 80;
    }
  */


  if (kadomode == 2 || kadomode == -2) {
    dir_move = 0;
  } else if (kadomode == 1) {
    if (dir_move > 90)dir_move = 90;
  } else if (kadomode == -1)
    if (dir_move < -90)dir_move = -90;
  //-----------------------------ライン
  if (check_line()) {
    cal_line();
    clr_timer(0);
  }

  if (!SW2) {
    while (!SW2);
    tone(buzzer, 2000, 100);
    keepermode = !keepermode;
  }


  if (keepermode) {
    if (linex > 3) {
      dir_move = -90;
    } else if (linex < -3) {
      dir_move = 90;
    } else {
      if (liney >= 4) {
        dir_move = 180;
      } else if (liney <= 0) {
        dir_move = 0;
      } else {
        if (ball.dir > 20) {
          dir_move = 90;
          if (liney < 2) {
            dir_move = 75;
          } else if (liney > 2) {
            dir_move = 105;
          }
        } else if (ball.dir < -20) {
          dir_move = -90;
          if (liney < 2) {
            dir_move = -75;
          } else if (liney > 2) {
            dir_move = -105;
          }
        } else {
          dir_move = 1000;
        }


      }

      if (linex > 0 && dir_move > 0) {
        dir_move = 1000;
      } else if (linex < 0 && dir_move < 0) {
        dir_move = 1000;
      }
    }
  }




  cal_motorspeed(dir_move, Speed);



  if (get_timer(32) > 200) {
    get_pixy();
    clr_timer(32);
  }

  if (!IFHOLD) {
    clr_timer(49);
  }
  if (ball.dir < 60 && ball.dir > -60 && seme.dir != 0 && IFHOLD && get_timer(49) > 1)
  {


    double Pc = 1;
    m_power[0] += seme.dir * Pc;
    m_power[1] -= seme.dir * Pc;
    m_power[2] -= seme.dir * Pc;
    m_power[3] += seme.dir * Pc;
  } else {
    dircontrol();
  }



  if (get_timer(0) < 100) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 255, 0);
    }
    neo_dir(dir_move, 255, 0, 0);
  }

  if (roll > 10 || roll < -10) {
    m_power[0] = 0;
    m_power[1] = 0;
    m_power[2] = 0;
    m_power[3] = 0;
    linex = 0;
    liney = 0;

    if (digitalRead(Pin_BUSY) == 1) {
#ifdef SE_ON
      myDFPlayer.play(10);
#endif
    }

    if (get_timer(7) > 200) {
      clr_timer(7);
      lite = !lite;
    }
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(lite * 255, 0, 0);
    }
  }



  pwm_out();

  /*if ((line[0] + line[1] + line[2] + line[3])*0 + line[4] + line[5] + line[6] + line[7] > 0) {
    tone(buzzer, 2000, 10);
    } else {
    noTone(buzzer);
    }
  */


  if (get_timer(9) > 250) {
    AA = !AA;
    digitalWrite(Pin_out1, AA);
    clr_timer(9);
  }
  if (BB != digitalRead(Pin_in1)) {
    clr_timer(10);
    BB = !BB;
  }
  if (get_timer(10) > 500) {
    conect = 0;
  } else {
    conect = 1;
  }




  FastLED.show();


  kicker();

  if (linex > 0 && linex < 3 || linex < 0 && linex > -3 || liney > 0 && liney < 3 || liney < 0 && liney > -3) {
    //tone(buzzer, 1800);
  } else {
    //noTone(buzzer);
  }

  if (linex != 0 || liney != 0) {
    //tone(buzzer,2000);
  } else {
    //noTone(buzzer);
  }

}