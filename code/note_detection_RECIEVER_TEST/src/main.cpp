#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

//struct of the recieved data
typedef struct struct_message {
  int tone;
} struct_message;

//#define SENDER
#define RECIEVER




#ifdef SENDER
#ifdef RECIEVER
#error can not define both SENDER and RECIEVER!
#endif


//////////////////////////////EPS_NOW
// the mac address we are sending to
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xA9, 0x8A, 0xA8};

// create instance of struct_message called myData
struct_message myData;

// callback funciton that will be executed when a message is sent. Here the funciton prints if the message was successfully delivered or not
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void send(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  // send                       the mac adress of reciever, the first element of myData list, size of myData
  
  // check if the message was sent successfully
  if (result == ESP_OK) { // if the result maches the ESP thumbs up value
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
//////////////////////////////ESP_NOW

//---------------------------------------------------------------------------//
int  in[128];
byte NoteV[13]={8,23,40,57,76,96,116,138,162,187,213,241,255};  //data for note detection based on frequency
float f_peaks[5]; // top 5 frequencies peaks in descending order
#define MIC_PIN 32   // change as per Microphone pin OBS 32 seems to be one of the few pins that works at the same time as wifi is active
#define ZERO_SHIFT 180 // initially set to 500
#define THRESHOLD 70 // initially set to 3
//---------------------------------------------------------------------------//


//-----------------------------FFT Function----------------------------------------------//
// Documentation on EasyFFT:https://www.instructables.com/member/abhilash_patel/instructables/
// EasyFFT code optimised for 128 sample size to reduce mamory consumtion
float FFT(byte N,float Frequency)
{
byte data[8]={1,2,4,8,16,32,64,128};
int a,c1,f,o,x;
a=N;  
                                 
      for(int i=0;i<8;i++)                 //calculating the levels
         { if(data[i]<=a){o=i;} }
      o=7;
byte in_ps[data[o]]={};     //input for sequencing
float out_r[data[o]]={};   //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
           
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
         {
          c1=data[b];
          f=data[o]/(c1+c1);
                for(int j=0;j<c1;j++)
                    { 
                     x=x+1;
                     in_ps[x]=in_ps[j]+f;
                    }
         }
 
      for(int i=0;i<data[o];i++)            // update input array as per bit reverse order
         {
          if(in_ps[i]<a)
          {out_r[i]=in[in_ps[i]];}
          if(in_ps[i]>a)
          {out_r[i]=in[in_ps[i]-a];}      
         }

int i10,i11,n1;
float e,c,s,tr,ti;

    for(int i=0;i<o;i++)                                    //fft
    {
     i10=data[i];              // overall values of sine cosine  
     i11=data[o]/data[i+1];    // loop with similar sine cosine
     e=6.283/data[i+1];
     e=0-e;
     n1=0;

          for(int j=0;j<i10;j++)
          {
          c=cos(e*j); 
          s=sin(e*j); 
          n1=j;
          
                for(int k=0;k<i11;k++)
                 {
                 tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                 ti=s*out_r[i10+n1]+c*out_im[i10+n1];
          
                 out_r[n1+i10]=out_r[n1]-tr;
                 out_r[n1]=out_r[n1]+tr;
          
                 out_im[n1+i10]=out_im[n1]-ti;
                 out_im[n1]=out_im[n1]+ti;          
          
                 n1=n1+i10+i10;
                  }       
             }
     }

//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
    for(int i=0;i<data[o-1];i++)               // getting amplitude from compex number
        {
         out_r[i]=sqrt((out_r[i]*out_r[i])+(out_im[i]*out_im[i])); // to  increase the speed delete sqrt
         out_im[i]=(i*Frequency)/data[o];
         /*
         Serial.print(out_im[i],2); Serial.print("Hz");
         Serial.print("\t");                            // uncomment to print freuency bin    
         Serial.println(out_r[i]); 
         */
        }

x=0;       // peak detection
   for(int i=1;i<data[o-1]-1;i++)
      {
      if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
      {in_ps[x]=i;    //in_ps array used for storage of peak number
      x=x+1;}    
      }

s=0;
c=0;
    for(int i=0;i<x;i++)             // re arraange as per magnitude
    {
        for(int j=c;j<x;j++)
        {
            if(out_r[in_ps[i]]<out_r[in_ps[j]]) 
                {s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;}
        }
    c=c+1;
    }
    
    for(int i=0;i<5;i++)     // updating f_peak array (global variable)with descending order
     {
     f_peaks[i]=(out_im[in_ps[i]-1]*out_r[in_ps[i]-1]+out_im[in_ps[i]]*out_r[in_ps[i]]+out_im[in_ps[i]+1]*out_r[in_ps[i]+1])
     /(out_r[in_ps[i]-1]+out_r[in_ps[i]]+out_r[in_ps[i]+1]);
     }
}
    
//------------------------------------------------------------------------------------//


//-----------------------------Tone Detection Function----------------------------------------------//
// Documentation on Tone_detection:https://www.instructables.com/member/abhilash_patel/instructables/
// Code Written By: Abhilash Patel
// Contact: abhilashpatel121@gmail.com
// This code written for arduino Nano board (should also work for UNO)
// This code won't work for any board having RAM less than 2kb,
void Tone_det()
{ long unsigned int a1,b,a2;
  float a;
  float sum1=0,sum2=0;
  float sampling;
  a1=micros();
        for(int i=0;i<128;i++)
          {
            //Serial.println("analogRead now");
            a=analogRead(MIC_PIN)-ZERO_SHIFT;     //rough zero shift
            //utilising time between two sample for windowing & amplitude calculation
            sum1=sum1+a;              //to average value
            sum2=sum2+a*a;            // to RMS value
            a=a*(sin(i*3.14/128)*sin(i*3.14/128));   // Hann window
            in[i]=10*a;                // scaling for float to int conversion
            delayMicroseconds(195);   // based on operation frequency range
          }
b=micros();

sum1=sum1/128;               // Average amplitude
sum2=sqrt(sum2/128);         // RMS
sampling= 128000000/(b-a1);  // real time sampling frequency

//for very low or no amplitude, this code wont start
//it takes very small aplitude of sound to initiate for value sum2-sum1>3, 
//change sum2-sum1 threshold based on requirement
if(sum2-sum1>THRESHOLD){  
       FFT(128,sampling);        
       //EasyFFT based optimised  FFT code, 
       //this code updates f_peaks array with 5 most dominent frequency in descending order
 
 for(int i=0;i<12;i++){in[i]=0;}  // utilising in[] array for further calculation

int j=0,k=0; //below loop will convert frequency value to note 
       for(int i=0; i<5;i++)
           {
           if(f_peaks[i]>1040){f_peaks[i]=0;}
           if(f_peaks[i]>=65.4   && f_peaks[i]<=130.8) {f_peaks[i]=255*((f_peaks[i]/65.4)-1);}
           if(f_peaks[i]>=130.8  && f_peaks[i]<=261.6) {f_peaks[i]=255*((f_peaks[i]/130.8)-1);}
           if(f_peaks[i]>=261.6  && f_peaks[i]<=523.25){f_peaks[i]=255*((f_peaks[i]/261.6)-1);}
           if(f_peaks[i]>=523.25 && f_peaks[i]<=1046)  {f_peaks[i]=255*((f_peaks[i]/523.25)-1);}
           if(f_peaks[i]>=1046 && f_peaks[i]<=2093)  {f_peaks[i]=255*((f_peaks[i]/1046)-1);}
           if(f_peaks[i]>255){f_peaks[i]=254;}
           j=1;k=0;
         
         while(j==1)
              {
              if(f_peaks[i]<NoteV[k]){f_peaks[i]=k;j=0;}
              k++;  // a note with max peaks (harmonic) with aplitude priority is selected
              if(k>15){j=0;}
              }

              if(f_peaks[i]==12){f_peaks[i]=0;}
              k=f_peaks[i];
              in[k]=in[k]+(5-i);
            }

k=0;j=0;
          for(int i=0;i<12;i++)
             {
              if(k<in[i]){k=in[i];j=i;}  //Max value detection
             }
       // Note print
       // if you need to use note value for some application, use of note number recomendded
       // where, 0=c;1=c#,2=D;3=D#;.. 11=B;      
       //a2=micros(); // time check
        k=j;
        // if(k==0) {Serial.println('C');}
        // if(k==1) {Serial.print('C');Serial.println('#');}
        // if(k==2) {Serial.println('D');}
        // if(k==3) {Serial.print('D');Serial.println('#');}
        // if(k==4) {Serial.println('E');}
        // if(k==5) {Serial.println('F');}
        // if(k==6) {Serial.print('F');Serial.println('#');}
        // if(k==7) {Serial.println('G');}
        // if(k==8) {Serial.print('G');Serial.println('#');}
        // if(k==9) {Serial.println('A');}
        // if(k==10){Serial.print('A');Serial.println('#');}
        // if(k==11){Serial.println('B');}
        if (k >= 0 && k <= 11) {
          myData.tone = k;
          Serial.print("playing: ");Serial.println(k);
          send();
        }
        
       }
}









void setup() {
  Serial.begin(115200);

  //////////////////////////////ESP_NOW
  WiFi.mode(WIFI_STA); // set the device as a Wi-Fi station
  //Initialize ESP-NOW:
  if (esp_now_init() != ESP_OK) {
      // esp_now_init() returns value of ESP_OK if initialization succeded, or other values if there was a faliure
    Serial.println("Error initializing ESP-NOW");
    return; // cancel the program?
  }

  // initialize a callback function to be called whenever a message is sent, here we use our function OnDataSent
  esp_now_register_send_cb(OnDataSent);

  // here we have to pair our board with another ESP-NOW device o send data
  //Register peer
  esp_now_peer_info_t peerInfo; // create an instance of the struct esp_now_peer_info, called peerInfo
                                // the struct contains ESPNOW peer information parameters
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); //copy the 6 element long array of the broadcastAddress to the peer_addr slot in peerInfo
  peerInfo.channel = 0; // slot of peerInfo: Wi-Fi channel that peer uses to send/receive ESPNOW data.
                        // If the value is 0, use the current channel which station or softap is on.
                        // Otherwise, it must be set as the channel that station or softap is on.
  peerInfo.encrypt = false; // do not encrypt the data to be sent

  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    // if we give the address of the esp_now_peer_info_t instance peerInfo to esp_now_add_peer and it does not return OK value
    Serial.println("Failed to add peer");
    return; // cancel
  }
  //////////////////////////////ESP_NOW
  Serial.println("here we go!");
}


void loop() 
{
  Tone_det();
}

#endif




#ifdef RECIEVER

///////////////////////////FASTLED
#include <FastLED.h>
///////////////////////////!FASTLED

// create an instance of struct_message called myData
struct_message myData;

////////////////////////////////TIMING
#define GLOBAL_FADE_TIME 3
#define DEBOUNCE_TIME 5
unsigned long prev_tone_time = 0;
int prev_tone = 0;
////////////////////////////////!TIMING




/////////////////////////////////////////////////////THE STRIPS OOP
// C
#define LED_PIN_C 32
#define NUM_LEDS_C 60
// Db
#define LED_PIN_Db 33
#define NUM_LEDS_Db 24
// D_Eb
#define LED_PIN_D_Eb 25
#define NUM_LEDS_D_Eb 22
// E_F_Gb
#define LED_PIN_E_F_Gb 26
#define NUM_LEDS_E_F_Gb 26
// G_Ab_A
#define LED_PIN_G_Ab_A 27
#define NUM_LEDS_G_Ab_A 26
// Eb_Bb
#define LED_PIN_Eb_Bb 14
#define NUM_LEDS_Eb_Bb 21
// B
#define LED_PIN_B 13
#define NUM_LEDS_B 24

// MODES MENU
#define LED_PIN_MENU 18
#define NUM_LEDS_MENU 8
CRGB ledstrip_menu[NUM_LEDS_MENU];
int menu_index = NUM_LEDS_MENU-1;
#define PIN_BUTTON_R 16
#define PIN_BUTTON_L 17
#define PIN_BUTTON_TOGGLE_RECEIVE 21    
#define PIN_LED_TOGGLE_RECEIVE 19
bool receive_state = true;


/////////////////////////////////////////////////////ANIMATION VARS
bool polizei_toggle = true;
int tmp_indx = 0;
//////////////////////////////////////////////////////



CRGB *array_of_LED_strips[] {
  new CRGB[NUM_LEDS_C],      // C
  new CRGB[NUM_LEDS_Db],     // Db
  new CRGB[NUM_LEDS_D_Eb],   // D_Eb
  new CRGB[NUM_LEDS_E_F_Gb], // E_F_Gb
  new CRGB[NUM_LEDS_G_Ab_A], // G_Ab_A
  new CRGB[NUM_LEDS_Eb_Bb],  // Eb_Bb
  new CRGB[NUM_LEDS_B]       // B
};

// class for a segment of a led strip
class LED_Segment {
  private:
    byte LED_strip;   // the LED strip that this segment belong to
    byte num_leds;    // num of leds in this segment
    byte start_index; // start index of this segment in LED_strip
    byte end_index;   // end index of this segment in LED_strip

    // variable for keeping timestamp of last time this segment was called to light up
    unsigned long prev_time = 0;

    // INTERNAL GETTER FOR THE RIGHT CRGB INSTANCE
    CRGB *get_LED_array() { // declare a variable that holds a pointer
      return array_of_LED_strips[LED_strip];
    }
  
  public:
    // CONSTRUCTOR
    // index in array_of_LED_strips, start index, num of leds
    LED_Segment(byte LED_strip_, byte start_index_, byte num_leds_) {
      LED_strip = LED_strip_;
      num_leds = num_leds_;
      start_index = start_index_;
      end_index = start_index + num_leds;
    }

    // FUNCTIONS




    void flash() {
      // happens with intervals of at lest DEBOUNCE_TIME ms
      if (millis()-prev_time > DEBOUNCE_TIME) {
        prev_time = millis();


        // Set all the leds in this LED segment to yellow
        // Serial.print("i = ");
        for (int i = start_index; i < end_index; i++) {
          // Serial.print(i); Serial.print(", ");
          get_LED_array()[i] = CRGB::Yellow;
        }
        // Serial.println(); Serial.println("done with for-loop in flash()");
        FastLED.show();
      }
    }

    void flash_red() {
      // happens with intervals of at lest DEBOUNCE_TIME ms
      if (millis()-prev_time > DEBOUNCE_TIME) {
        prev_time = millis();


        // Set all the leds in this LED segment to yellow
        // Serial.print("i = ");
        for (int i = start_index; i < end_index; i++) {
          // Serial.print(i); Serial.print(", ");
          get_LED_array()[i] = CRGB::Red;
        }
        // Serial.println(); Serial.println("done with for-loop in flash()");
        FastLED.show();
      }
    }
    void flash_blue() {
      // happens with intervals of at lest DEBOUNCE_TIME ms
      if (millis()-prev_time > DEBOUNCE_TIME) {
        prev_time = millis();


        // Set all the leds in this LED segment to yellow
        // Serial.print("i = ");
        for (int i = start_index; i < end_index; i++) {
          // Serial.print(i); Serial.print(", ");
          get_LED_array()[i] = CRGB::Blue;
        }
        // Serial.println(); Serial.println("done with for-loop in flash()");
        FastLED.show();
      }
    } 
    
};


class Button {
  private:
    byte pin;
    byte state;
    byte last_reading;
    unsigned long last_debounce_time = 0;
    unsigned long debounce_delay = 50;
    unsigned long last_pressed_time = 0;
    unsigned long press_delay = 200;
  public:
    Button(byte pin) {
      this->pin = pin;
      last_reading = LOW;
      init();
    }
    void init() {
      pinMode(pin, INPUT);
      update();
    }
    void update() {
      byte new_reading = digitalRead(pin);
      if (new_reading != last_reading) {
        last_debounce_time = millis();
      }
      if(millis() - last_debounce_time > debounce_delay) {
        // Update the 'state' attribute only if debounce is checked
        // Serial.println("updating state!");
        state = new_reading;
      }
      last_reading = new_reading;
    }
    byte get_state() {
      update();
      return state;
    }
    bool is_pressed() {
      // Serial.println("checking button!");
      if (get_state() && millis() - last_pressed_time > press_delay) {
        last_pressed_time = millis();
        return HIGH;
      }
      else {
        return LOW;
      }
    }
};



////////////////////CREATE THE OBJECTS IN ARRAY
LED_Segment array_of_segments[] = {
  // index in array_of_LED_strips, start index, num of leds
  LED_Segment(0, 0, NUM_LEDS_C),  // 0 : C
  LED_Segment(1, 0, NUM_LEDS_Db), // 1 : Db
  LED_Segment(2, 12, 10),         // 2 : D
  LED_Segment(2, 0, 12),          // 3a: Eb <--
  LED_Segment(3, 14, 12),         // 4 : E
  LED_Segment(3, 7, 7),           // 5 : F
  LED_Segment(3, 0, 7),           // 6 : Gb
  LED_Segment(4, 0, 7),           // 7 : G
  LED_Segment(4, 7, 7),           // 8 : Ab
  LED_Segment(4, 14, 12),         // 9 : A
  LED_Segment(5, 0, 12),          // 3b: Eb <--
  LED_Segment(5, 12, 9),          // 10: Bb     (removed one led here, thus 9 not 10)
  LED_Segment(6, 0, NUM_LEDS_B)   // 11: B
};
//////////////////////////////////////////////////////!THE STRIPS OOP

// BUTTON OBJECTS:
Button button_L(PIN_BUTTON_L);
Button button_R(PIN_BUTTON_R);
Button button_recieve(PIN_BUTTON_TOGGLE_RECEIVE);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
                // import mac value as a pointer
                // import incomingData value as a pointer
                // import len as an integer
  memcpy(&myData, incomingData, sizeof(myData));
        // copy incompingData array to the address of myData (has to specify the size) first element 
  // print all the recieved data
  //Serial.println(len);
  // Serial.print("Char: ");
  Serial.print("tone: ");
  Serial.println(myData.tone);

  ///////////////////////////FASTLED
  if (receive_state) {
    if(myData.tone < 13 || myData.tone >= 0) {
      Serial.println("flash!");
      array_of_segments[myData.tone].flash();
    }
  }
  ///////////////////////////!FASTLED
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  
  delay(300); //trying to elimintate flash

  // initialize ESP-NOW and check if there is an error (in that case cancel the setup)
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received.
  esp_now_register_recv_cb(OnDataRecv);

  ///////////////////////////FASTLED
  // FastLED.addLeds<WS2812B, LED_PIN_C, GRB>(leds_C, NUM_LEDS_C);
  // FastLED.addLeds<WS2812B, LED_PIN_Db_D_Eb, GRB>(leds_Db_D_Eb, NUM_LEDS_Db_D_Eb);

  FastLED.addLeds<WS2812B, LED_PIN_C, GRB>(array_of_LED_strips[0], NUM_LEDS_C);
  FastLED.addLeds<WS2812B, LED_PIN_Db, GRB>(array_of_LED_strips[1], NUM_LEDS_Db);
  FastLED.addLeds<WS2812B, LED_PIN_D_Eb, GRB>(array_of_LED_strips[2], NUM_LEDS_D_Eb);
  FastLED.addLeds<WS2812B, LED_PIN_E_F_Gb, GRB>(array_of_LED_strips[3], NUM_LEDS_E_F_Gb);
  FastLED.addLeds<WS2812B, LED_PIN_G_Ab_A, GRB>(array_of_LED_strips[4], NUM_LEDS_G_Ab_A);
  FastLED.addLeds<WS2812B, LED_PIN_Eb_Bb, GRB>(array_of_LED_strips[5], NUM_LEDS_Eb_Bb);
  FastLED.addLeds<WS2812B, LED_PIN_B, GRB>(array_of_LED_strips[6], NUM_LEDS_B);
  FastLED.addLeds<WS2812B, LED_PIN_MENU, GRB>(ledstrip_menu, NUM_LEDS_MENU);
  FastLED.setBrightness(20); // 40
  ///////////////////////////!FASTLED


  /////////////////////////////////MENU
  pinMode(PIN_LED_TOGGLE_RECEIVE, OUTPUT);
  digitalWrite(PIN_LED_TOGGLE_RECEIVE, LOW);
  /////////////////////////////////!MENU
}

void polizei() {
  EVERY_N_MILLISECONDS(500) {
    if (polizei_toggle) {
      for (int i = 0; i < 60; i++) {
        if (i < 30) {
          array_of_LED_strips[0][i] = CRGB::Red;
        } else {
          array_of_LED_strips[0][i] = CRGB::Blue;
        }
      }

      for (int i = 1; i < 7; i++) {
        array_of_segments[i].flash_red();
      }
      for (int i = 7; i < 13; i++) {
        array_of_segments[i].flash_blue();
      }
    } else {

      for (int i = 0; i < 60; i++) {
        if (i < 30) {
          array_of_LED_strips[0][i] = CRGB::Blue;
        } else {
          array_of_LED_strips[0][i] = CRGB::Red;
        }
      }
      for (int i = 1; i < 7; i++) {
        array_of_segments[i].flash_blue();
      }
      for (int i = 7; i < 13; i++) {
        array_of_segments[i].flash_red();
      }    
    }
    polizei_toggle = !polizei_toggle;
  }
}

void spin() {
  EVERY_N_MILLISECONDS(100) {
    if (tmp_indx < 0 || tmp_indx > 12) {
      tmp_indx = 0;
    }
    array_of_segments[tmp_indx].flash();
    tmp_indx ++;
  }
}

void update_menu_state() {
  if (button_R.is_pressed()) {
    if (menu_index < NUM_LEDS_MENU-1) {
      menu_index++;
    }
  } else if (button_L.is_pressed()) {
    if (menu_index > 0) {
      menu_index--;
    }
  } else if (button_recieve.is_pressed()) {
    receive_state = !receive_state;
  }

  if (receive_state) {
    digitalWrite(PIN_LED_TOGGLE_RECEIVE, HIGH);
  } else {
    digitalWrite(PIN_LED_TOGGLE_RECEIVE, LOW);
  }
  fadeToBlackBy(ledstrip_menu, NUM_LEDS_MENU, 50);
  ledstrip_menu[menu_index] = CRGB::Yellow;
}

void loop(){

  EVERY_N_MILLISECONDS(1000) {
    Serial.println("loop running");
  }


  
  update_menu_state();


  if (!receive_state) {

    switch(menu_index) {
      case 7:
      EVERY_N_MILLISECONDS(1000) {
        array_of_segments[0].flash();
      }
      break;

      case 6:
        spin();
      break;

      case 5:
        polizei();
      break;

      case 4:
        //
      break;

      case 3:
        //
      break;

      case 2:
        //
      break;

      case 1:
        //
      break;

      case 0:
        //
      break;
    }
  } else {
    // receive mode!
  }

  
  fadeToBlackBy(array_of_LED_strips[0], NUM_LEDS_C, GLOBAL_FADE_TIME); // gradually fade out all the ledstrips
  fadeToBlackBy(array_of_LED_strips[1], NUM_LEDS_Db, GLOBAL_FADE_TIME);
  fadeToBlackBy(array_of_LED_strips[2], NUM_LEDS_D_Eb, GLOBAL_FADE_TIME);
  fadeToBlackBy(array_of_LED_strips[3], NUM_LEDS_E_F_Gb, GLOBAL_FADE_TIME);
  fadeToBlackBy(array_of_LED_strips[4], NUM_LEDS_G_Ab_A, GLOBAL_FADE_TIME);
  fadeToBlackBy(array_of_LED_strips[5], NUM_LEDS_Eb_Bb, GLOBAL_FADE_TIME);
  fadeToBlackBy(array_of_LED_strips[6], NUM_LEDS_B, GLOBAL_FADE_TIME);

  // UPDATE ALL LEDS
  FastLED.show();
}
#endif

