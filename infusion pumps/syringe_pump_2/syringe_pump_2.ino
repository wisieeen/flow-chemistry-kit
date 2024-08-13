/*
length of 10mL syringe is ~5cm, this means ~80000 steps.
1 step is 0.000125mL=0.125uL=125nL
note that in program, steps must be multipled by 2


*/

#include <credentials.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "ArduinoJson.h"

const char* mqtt_server = "10.0.0.4";
WiFiClient esp_pump_2;
PubSubClient client(esp_pump_2);
const unsigned long long_max = 4294967295;
const byte pin_1_s = 12; //step pin
const byte pin_1_d = 14; //dir pin
const byte pin_1_e = 27; //endstop pin
const byte pin_1_com = 32; //pin for communication between boards

int speed_1 = 70; //70 is minimum
int block_num = 0;
const int block_size = 10;
int phase_1 = 0;
unsigned long timer_1 = 0;
unsigned long next_step_1 = 0; //when stepper moves
unsigned long block_start = 0; //when program block started
unsigned long block_stop = 0;
long absolute_pos_1 = 0;
long requested_pos_1 = 0;
int direction_1 = 1;
int program_length = 0;
JsonDocument program_1; //program buffer ["step1":["delay":70,"micros":1000],"step2":...]
JsonDocument program_0;//executed program
bool run_program_switch = 0;
/*=====================
function declarations for connectivity
=====================*/


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(mySSID);
  //WiFi.setSleepMode(WIFI_NONE_SLEEP); //should prevent random disconnects > https://github.com/esp8266/Arduino/issues/5083
  WiFi.mode(WIFI_STA); //WiFi mode station (connect to wifi router only
  WiFi.begin(mySSID, myPASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String ttopic = String(topic);
  byte p[length];
  for (int i=0;i<length;i++) {
    p[i] = payload[i];
  }
  String pp= (char*)p;
  pp = pp.substring(0,length); //w/o this some weird data is added to stirng
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print("; length: ");
  Serial.print(length);
  Serial.print("; payload: ");
  Serial.println(pp);

  if (ttopic=="pump_2/speed"){ //actually time distance between steps
    speed_1 = pp.toInt();
    Serial.print("pump/speed activated: ");
    Serial.println(speed_1);
  }

  else if (ttopic=="pump_2/steps"){
    requested_pos_1 += pp.toInt()*2;
    if(requested_pos_1 >absolute_pos_1){
      direction_1 = 1;
      digitalWrite(pin_1_d,1);
    }
    else if(requested_pos_1 <absolute_pos_1){
      direction_1 = -1;
      digitalWrite(pin_1_d,0);
    }
    next_step_1 = micros();
  }
  
  else if (ttopic=="pump_2/end_position"){
    requested_pos_1 = pp.toInt()*2;

    if(requested_pos_1 >absolute_pos_1){
      direction_1 = 1;
      digitalWrite(pin_1_d,1);
    }
    else if(requested_pos_1 <absolute_pos_1){
      direction_1 = -1;
      digitalWrite(pin_1_d,0);
    }
  }

  else if (ttopic=="pump_2/run"){
    next_step_1 = micros();
    Serial.print("pump will run in next_step_1: ");
    Serial.println(next_step_1);
  }

  else if (ttopic=="pump_2/run_program"){
    program_0 = program_1;
    speed_1 = int(program_0["0"][0]);
    if (speed_1 <0){
      direction_1 = 1;
      digitalWrite(pin_1_d,1);
    }
    else{
      direction_1 = -1;
      digitalWrite(pin_1_d,0);
    }
    speed_1 = abs(speed_1);
    run_program_switch = 1;
    block_num = 1;
    //client.publish("pump_master/program_request","1");
    Serial.println("pump will run program");
    while(!digitalRead(pin_1_com)){
      ;
    }
    next_step_1 = micros();
    block_start = next_step_1;
    block_stop = block_start + int(program_0["0"][1]);
  }
  else if (ttopic=="pump_2/program"){
    deserializeJson(program_1, pp); //Parse message
    //Serial.print("program received: ");
    //Serial.println(int(program_1["0"][0]));
  }

  else if (ttopic=="pump_2/homing"){
    homing();
  }

  else if (ttopic=="pump_2/program_length"){
    program_length = pp.toInt();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp_pump_2")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("common/status","syringe pump_2 connected");
      char* a = (char*)absolute_pos_1;
      client.publish("pump_2/position_now",a);
      // ... and resubscribe
      client.subscribe("pump_2/speed");
      client.subscribe("pump_2/steps");
      client.subscribe("pump_2/end_position");
      client.subscribe("pump_2/program");
      client.subscribe("pump_2/run_program");
      client.subscribe("pump_2/run");
      client.subscribe("pump_2/homing");
      client.subscribe("pump_2/program_length");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*=====================
function declarations for pumping
=====================*/

void pump_service(){
  unsigned long tn = micros();
  if(run_program_switch == 1){ //checks if it is program run
    //Serial.println("program runs");
    if(tn <= block_stop){ //checks if block isn't done
      //Serial.println("block continues");
      if(tn>=next_step_1){ //checks if next step now
        //Serial.println("next step");
        phase_1 = abs(phase_1-1); //single step
        digitalWrite(pin_1_s,phase_1);
        next_step_1 += speed_1;
        absolute_pos_1 += direction_1;
      } //end of this step
    }//end of present block
    else{ //if block is done
      //Serial.println("next block");
      if (block_num >= program_length){//checks if it was last block
        //Serial.println("last block done");
        next_step_1 = long_max;
        run_program_switch = 0;
        char cstr[16];
        itoa(absolute_pos_1/2, cstr, 10);        
        client.publish("pump_2/position_now",cstr);
      }
      else{
        block_start = tn;
        int block_num_mod = block_num % 10;
        /*Serial.print("block_num_mod: ");
        Serial.print(block_num_mod);
        Serial.print(" block_start: ");
        Serial.print(block_start);*/
        block_stop = block_start + int(program_0[String(block_num)][1]);
        speed_1 = int(program_0[String(block_num)][0]);
        /*Serial.print(" block_stop: ");
        Serial.print(block_stop);
        Serial.print(" delta t: ");
        Serial.print(block_stop-block_start);
        Serial.print(" speed1: ");
        Serial.println(speed_1);*/
        if (speed_1 <0){//sets direction
          direction_1 = 1;
          digitalWrite(pin_1_d,1);
        }
        else{
          direction_1 = -1;
          digitalWrite(pin_1_d,0);
        }
        speed_1 = abs(speed_1);
        if (block_num_mod == (char)(block_size-1)){ //if block number divisible by 9, then replace old buffer with new one
          program_0 = program_1;
          //Serial.println("buffer swap");
          //serializeJson(program_0, Serial);
          //client.publish("pump_master/program_request","1");
        }
        else if(block_num_mod == 1){ //ask for next village (of blocks)
          //Serial.println("buffer request");
          client.publish("pump_2/program_request","1");
        }
        block_num++;
      }
    }
  } //end of program run
    
  else if(tn>=next_step_1){
    if(absolute_pos_1 == requested_pos_1){ 
      //if goal reached, write timing info and sets next step to distant number
      char cstr[16];
      itoa(absolute_pos_1/2, cstr, 10);
      client.publish("pump_2/position_now",cstr);
      //if (series_length >0){
      //  series_length--;
      //  while (digitalRead(pin_1_com)==1){;}
      //  client.publish("pump_master/step_done","Y");
      //}
      next_step_1 = long_max;}
    else{
      phase_1 = abs(phase_1-1);
      //Serial.print(phase_1);
      digitalWrite(pin_1_s,phase_1);
      next_step_1 += speed_1;
      absolute_pos_1 += direction_1;
      
    }
  }
}
//client.publish("pump_master/step_done","Y");
void homing(){
  digitalWrite(pin_1_d,1);
  for(int i=0; i<6000;i++){ //initial retract, just in case
    digitalWrite(pin_1_s,1);
    delayMicroseconds(70);
    digitalWrite(pin_1_s,0);
    delayMicroseconds(70);
  }
  digitalWrite(pin_1_d,0);
  while(digitalRead(pin_1_e)!=0){ //crude homing
    digitalWrite(pin_1_s,1);
    delayMicroseconds(70);
    digitalWrite(pin_1_s,0);
    delayMicroseconds(70);
  }
  digitalWrite(pin_1_d,1);
  for(int i=0; i<1000;i++){ //retract
    digitalWrite(pin_1_s,1);
    delayMicroseconds(70);
    digitalWrite(pin_1_s,0);
    delayMicroseconds(70);
  }
  digitalWrite(pin_1_d,0);
  while(digitalRead(pin_1_e)!=0){ //precise homing run
    digitalWrite(pin_1_s,1);
    delayMicroseconds(500);
    digitalWrite(pin_1_s,0);
    delayMicroseconds(500);
  }
  absolute_pos_1 = 0;
  client.publish("pump_2/position_now","0");
}

/*=====================
program executions
=====================*/

void setup(){
  pinMode(pin_1_s, OUTPUT);
  pinMode(pin_1_d, OUTPUT);
  pinMode(pin_1_e, INPUT_PULLUP);
  pinMode(pin_1_com, INPUT);
  Serial.begin(2000000);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  timer_1=micros();
}

void loop(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  pump_service();
}
