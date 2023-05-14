#include "Arduino.h"

#define LINEF 250   //ラインしきい値
#define MAXspeed 100

#define Pin_kicker 24
#define Pin_hold A8 //22

void kicker(bool q){
  digitalWrite(Pin_kicker,q);
}



#define Pin_ball1 A3
#define Pin_ball2 A2
#define Pin_ball3 A5
#define Pin_ball4 A4
#define Pin_ball5 A7
#define Pin_ball6 A6
#define Pin_ball7 A1
#define Pin_ball8 A0

#define Pin_line1 42
#define Pin_line2 43
#define Pin_line3 40
#define Pin_line4 A12
#define Pin_line5 46
#define Pin_line6 47
#define Pin_line7 44
#define Pin_line8 A13
#define Pin_line9 38
#define Pin_line10 39
#define Pin_line11 36
#define Pin_line12 A10
#define Pin_line13 34
#define Pin_line14 35
#define Pin_line15 32
#define Pin_line16 A11

#define Pin_motor1F 3
#define Pin_motor1B 5
#define Pin_motor2F 8
#define Pin_motor2B 12
#define Pin_motor3F 2
#define Pin_motor3B 6
#define Pin_motor4F 7
#define Pin_motor4B 11

#define Pin_in1 13
#define Pin_in2 15
#define Pin_in3 17
#define Pin_in4 19

#define Pin_out1 31
#define Pin_out2 29
#define Pin_out3 27
#define Pin_out4 25

#define Pin_BUSY 50

//------------------------------------------------------------------
//TWILITE DIP  (bluetoothモジュール)

int comin(int pin){              //データを受信
  if(pin==1)return digitalRead(Pin_in1);
  if(pin==2)return digitalRead(Pin_in2);
  if(pin==3)return digitalRead(Pin_in3);
  if(pin==4)return digitalRead(Pin_in4);
}

void comout(int pin,int val){    //データを送信
  if(pin==1)digitalWrite(Pin_out1,val);
  if(pin==2)digitalWrite(Pin_out2,val);
  if(pin==3)digitalWrite(Pin_out3,val);
  if(pin==4)digitalWrite(Pin_out4,val);
}
//------------------------------------------------------------------



//押しボタン
#define SW1 digitalRead(30)
#define SW2 digitalRead(28)
#define SW3 digitalRead(10)


//トグルスイッチ
#define TS digitalRead(4)


//捕捉センサ　判定
#define IFHOLD (analogRead(Pin_hold) > 850)

//ブザーピン番号
#define buzzer 16










//----------------------------------------------------------------
//class


class BALL {
  public:
    int value[8]; //前から時計回りに 0,1,2,3,...7
    int distance;
    int dir;
    int n;
};

class GOAL {
  public:
    int color;
    int dir;
    int x;
    int y;
    int h;
    int w;
    bool cansee;
};



class MOTOR {
  public:
    void stop() {

    }
};




//----------------------------------------------------------------


#define _length 200

class TIMER{
  public:
    void reset();
    unsigned long get();
  private:
    unsigned long s_tim;
};


unsigned long TIMER::get(){
  return millis()-s_tim;
}

void TIMER::reset(){
  s_tim=millis();
}

class SOUND{
  public:
    void put(unsigned int f, unsigned int d);
    void put(unsigned int frequency);
    void play(unsigned int f, unsigned int d);
    unsigned int freq_now;

    void windowsXP();    //windowsXP起動音
    void tondemowonders();
  private:
    const int _pin = buzzer;
    int _Hz[7]={1077,1209,1357,1438,1614,1812,2033};
}buz;

void SOUND::put(unsigned int f, unsigned int d){
  tone(_pin,f,d);
}

void SOUND::put(unsigned int f){
  tone(_pin,f);
  freq_now = f;
}

void SOUND::play(unsigned int f, unsigned int d){
  tone(_pin,f,d*0.8);
  delay(d);
}

void SOUND::windowsXP(){
  play(1281,300);  //ミ♭(高)
  play(640,100);    //ミ♭(低)
  play(960,200);   //シ♭
  play(855,200);    //ラ♭
  play(640,200);    //ミ♭(低)
  play(1281,200);  //ミ♭(高)
  play(960,600);   //シ♭
}

void SOUND::tondemowonders(){
  play(_Hz[6]/2,_length);
  play(_Hz[4]/2,_length/2);
  play(_Hz[4]/2,_length/2);
  play(_Hz[6]/2,_length);
  play(_Hz[4]/2,_length/2);
  play(_Hz[4]/2,_length/2);
  play(_Hz[2]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[2],_length);
  play(_Hz[1],_length*2);
  play(_Hz[6]/2,_length/2);
  play(_Hz[5]/2,_length/2);
  play(_Hz[4]/2,_length);
  play(_Hz[5]/2,_length*2);
  play(_Hz[4]/2,_length);
  delay(_length);
  play(_Hz[1]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[1]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[5]/2,_length);
  play(_Hz[6]/2,_length);
  play(_Hz[0],_length);
  play(_Hz[1],_length);
  play(_Hz[6]/2,_length);
  play(_Hz[5]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[6]/2,_length*4);
  delay(_length*2);

  play(_Hz[6]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[6]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[2]/2,_length);
  play(_Hz[4]/2,_length);
  play(_Hz[2],_length);
  play(_Hz[1],_length*2);
  play(_Hz[6]/2,_length/2);
  play(_Hz[5]/2,_length/2);
  play(_Hz[4]/2,_length);
  play(_Hz[5]/2,_length);
  play(_Hz[5]/2,_length);
  play(_Hz[4]/2,_length);
  delay(_length);
  play(_Hz[6]/2,_length);
  play(_Hz[0],_length);
  play(_Hz[0],_length*2);
  play(_Hz[6]/2,_length);
  play(_Hz[0],_length);
  play(_Hz[0],_length);
  play(_Hz[5]/2,_length);
  play(_Hz[2],_length*2);
  play(_Hz[1],_length);
  play(_Hz[6]/2,_length);
  play(_Hz[1],_length*2);
  play(_Hz[1],_length);
  play(_Hz[2],_length*2);

}