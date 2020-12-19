/**
 * @file main.cpp
 * @author Electromatic (electromatic12@gmail.com)
 * @brief This programm lets you controll RGB led strips for generating light effects,
 *        featuring an audio response animation for every color channel defined in a 
 *        range of frequencies. It also does fade and strobe. Be sure tu use a microcontroller
 *        than can process FIR filters quick.
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Arduino.h>
#include <FIR.h>

//Signal processing related variables
uint16_t adc_Val;
uint16_t noise_gate=0;
uint16_t dcPoint = 470;

//Pin definitions
uint8_t G_Pin = D6;
uint8_t R_Pin = D5;
uint8_t B_Pin = D7;

//Output PWM variables
uint16_t r_val = 0;
uint16_t g_val = 0; 
uint16_t b_val = 0;

//Auxiliary gains for the different output filters
float low_gain=4.0;
float mid_gain=2.0;
float high_gain=2.0;
float adc_gain = 2.0;

//Timing related variables
double currentMicros, lastMicros;
double period;

bool strobeState = false;
bool linear_start_done=false;

typedef enum{
  red,
  green,
  blue
}sine_single_active_color;
sine_single_active_color sine_active_color;

bool red_state = true;
bool green_state, blue_state;

void write_all_pwm(uint16_t pwm_val);

unsigned long bauds = 115200;

/*
FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44100 Hz

* 0 Hz - 150 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -29.478551010401308 dB

* 2000 Hz - 15000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 18.70588638057564 dB

*/

#define MAIN_FILTER_TAP_NUM 5

static float main_filter[MAIN_FILTER_TAP_NUM] = {
  -0.7078311420671538,
  0.5990047231749758,
  0.2502128064992716,
  0.5990047231749758,
  -0.7078311420671538
};
FIR<float, MAIN_FILTER_TAP_NUM> main_filter_def;

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44100 Hz

* 0 Hz - 500 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 0.6148018070331738 dB

* 2000 Hz - 20000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -56.47895579042171 dB

*/

#define LOW_FILTER_TAP_NUM 20
static float low_filter[LOW_FILTER_TAP_NUM] = {
0.005945050333037109,0.008325667336150974,0.014864657590908685,0.025426273724393985,
0.039221803467893813,0.054890482142205495,0.070678862610494567,0.084691732680144308,
0.095174535069899868,0.100780935044871053,0.100780935044871053,0.095174535069899896,
0.084691732680144308,0.070678862610494567,0.054890482142205502,0.039221803467893841,
0.025426273724394002,0.014864657590908667,0.008325667336150972,0.005945050333037109
};
FIR<float, LOW_FILTER_TAP_NUM> low_filter_def;

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 44100 Hz

* 0 Hz - 200 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.66724371016172 dB

* 800 Hz - 3000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.856497020629501 dB

* 5000 Hz - 20000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.66724371016172 dB

*/

#define MID_FILTER_TAP_NUM 23
double mid_filter[MID_FILTER_TAP_NUM] = {
-0.000094577097179905,-0.000995711859796016,-0.003076430819006171,-0.007361443797606391,
-0.014599322684860186,-0.024994461219551550,-0.038048835776740619,-0.052563639830167638,
-0.066812206787690523,-0.078852371737755500,-0.086909844318388343,0.910452821922022593,
-0.086909844318388343,-0.078852371737755486,-0.066812206787690565,-0.052563639830167673,
-0.038048835776740626,-0.024994461219551557,-0.014599322684860194,-0.007361443797606399,
-0.003076430819006171,-0.000995711859796016,-0.000094577097179905
};
FIR<double, MID_FILTER_TAP_NUM> mid_filter_def;

//Sine (0 - PI) lookup table
uint16_t sinePointerAux;
#define SIN_TAPS 256
uint16_t sine_lookup[SIN_TAPS]{
0,
12,25,37,50,62,75,87,100,112,125,137,150,
162,174,187,199,211,224,236,248,260,272,284,296,
308,320,332,344,356,368,379,391,403,414,426,437,
448,459,471,482,493,504,515,525,536,547,557,568,
578,589,599,609,619,629,639,648,658,668,677,687,
696,705,714,723,732,740,749,757,766,774,782,790,
798,806,814,821,829,836,843,850,857,864,870,877,
883,890,896,902,908,913,919,924,930,935,940,945,
949,954,958,963,967,971,975,978,982,985,989,992,
995,998,1000,1003,1005,1007,1010,1011,1013,1015,1016,1018,
1019,1020,1021,1021,1022,1022,1022,1023,1022,1022,1022,1021,
1021,1020,1019,1018,1016,1015,1013,1011,1010,1007,1005,1003,
1000,998,995,992,989,985,982,978,975,971,967,963,
958,954,949,945,940,935,930,924,919,913,908,902,
896,890,883,877,870,864,857,850,843,836,829,821,
814,806,798,790,782,774,766,757,749,740,732,723,
714,705,696,687,677,668,658,648,639,629,619,609,
599,589,578,568,557,547,536,525,515,504,493,482,
471,459,448,437,426,414,403,391,379,368,356,344,
332,320,308,296,284,272,260,248,236,224,211,199,
187,174,162,150,137,125,112,100,87,75,62,50,
37,25,12
};


#define auxSineArraySize 2048
float auxSineArray[auxSineArraySize];
uint32_t taps;

void sine_array(float *s_array, uint16_t arraySize,double f, double fs, double max, double dc){
  Serial.println("Iniciando:");

  taps = fs/f;
  uint32_t auxCounter;
  float aux;

  for(auxCounter = 0; auxCounter < taps && auxCounter < arraySize; auxCounter++){
    aux = max*sin(2*PI*auxCounter/taps) + dc;
    //Serial.println(aux);
    s_array[auxCounter] = aux;
  }
}

//Sequences enumeration
typedef enum{
  audio,
  sine,
  sine_phaser,
  linear,
  strobe
}light_sequence;
light_sequence sequence;

/**
 * @brief Writes the corresponding RGB channels with the desired values
 * @param r 
 * @param g 
 * @param b 
 */
void write_rgb(uint16_t r, uint16_t g,uint16_t b){
  analogWrite(R_Pin, r);
  analogWrite(G_Pin, g);
  analogWrite(B_Pin, b);
}

/**
 * @brief Substracts the dcPoint value measured with no signal to the incoming signal
 *        We can also apply gain after. Returns the absolute value of the process if
 *        resulting signal is over the noise gate value, otherwise returns 0.
 * @param signal 
 * @param gain 
 * @return float 
 */
float process_Signal(float signal, float gain){
  float auxAbs = abs(signal-dcPoint)*gain;
  return auxAbs > noise_gate ? auxAbs : 0.0;
}

void audioSequence(){

  //Sampling period for 44.1Khz Fs
  if(currentMicros - lastMicros > 22.676){

    adc_Val = analogRead(A0);

    //This might add extra delay because of processing time 
    r_val = low_filter_def.processReading(adc_Val);
    g_val = mid_filter_def.processReading(adc_Val);

    //Signal processing and explicit conversion
    adc_Val = (uint16_t)process_Signal(adc_Val, adc_gain);
    
    r_val = (uint16_t)process_Signal(r_val,low_gain);
    g_val = (uint16_t)process_Signal(g_val,mid_gain);
    //b_val = adc_Val;
    if (r_val > 200) g_val = g_val * 0.2; 
    //Write every color channel with the corresponding signal
    write_rgb(r_val, g_val, 0);
    
    lastMicros = currentMicros;
  }
  currentMicros = micros();
}

bool linearStartUp(void){
  r_val = 42;
  g_val = 84;
  b_val = 126;
  return true;
}

void sinePhaser(){

  if (linear_start_done == false){
    linear_start_done=linearStartUp();
  }
  if(currentMicros - lastMicros > 4000){
    if(r_val > SIN_TAPS-1){
      r_val = 0;
      analogWrite(R_Pin, (sine_lookup[r_val++]));
    }else{
      analogWrite(R_Pin, (sine_lookup[r_val++]));
      if(r_val % 2 == 0){
        if(g_val > SIN_TAPS-1){
          g_val = 0;
          analogWrite(G_Pin, (sine_lookup[g_val++]));
        }else{
          analogWrite(G_Pin, (sine_lookup[g_val++]));
          if(g_val % 4 == 0){
            if(b_val > SIN_TAPS-1){
              b_val = 0;
              analogWrite(B_Pin, (sine_lookup[b_val++]));
            }else{
              analogWrite(B_Pin, (sine_lookup[b_val++]));
            }
          }
        }
      }
    }
    Serial.print(sine_lookup[r_val]);
    Serial.print(",");
    Serial.print(sine_lookup[g_val]);
    Serial.print(",");
    Serial.print(sine_lookup[b_val]);
    Serial.println();
    lastMicros = currentMicros;
  }
  currentMicros = micros();
}

bool sineSequence(bool r, bool g, bool b){
  bool return_val = false;
  if (linear_start_done == false){
    linear_start_done=linearStartUp();
  }
  if(currentMicros - lastMicros > 10000){
    if(r_val > SIN_TAPS-1){
      r_val = 0;
      return_val = true;
    }
    if(g_val > SIN_TAPS-1){
      g_val = 0;
      return_val = true;
    }
    if(b_val > SIN_TAPS-1){
      b_val = 0;
      return_val = true;
    }

    Serial.print(sine_lookup[r_val]);
    Serial.print(",");
    Serial.print(sine_lookup[g_val]);
    Serial.print(",");
    Serial.print(sine_lookup[b_val]);
    Serial.println();
    //write_all_pwm((sine_lookup[r_val++]));
    r == true ? analogWrite(R_Pin, (sine_lookup[r_val++])) : analogWrite(R_Pin,0);
    g == true ? analogWrite(G_Pin, (sine_lookup[g_val++])) : analogWrite(G_Pin,0);
    b == true ? analogWrite(B_Pin, (sine_lookup[b_val++])) : analogWrite(B_Pin,0);
    lastMicros = currentMicros;
  }
  currentMicros = micros();
  return return_val;
}

void strobeSequence(){
  if(currentMicros - lastMicros > 40000){
    if(strobeState){
      write_all_pwm(1023);
      strobeState = false;
    }else{
      write_all_pwm(0);
      strobeState = true;
    } 
    lastMicros = currentMicros;
  }
  
  currentMicros = micros();
}

void linearSequence(){
  if(sineSequence(red_state, green_state, blue_state)){
    switch(sine_active_color){
      case red:
      green_state = true;
      blue_state = false;
      red_state = false;
      sine_active_color = green;
      break;

      case green:
      green_state = false;
      blue_state = true;
      red_state = false;
      sine_active_color = blue;
      break;

      case blue:
      green_state = false;
      blue_state = false;
      red_state = true;
      sine_active_color = red;
      break;

      default:
      break;
    }
  }
}

/**
 * @brief Writes all PWM channels with the same value
 * @param pwm_val 
 */
void write_all_pwm(uint16_t pwm_val){
  analogWrite(R_Pin, pwm_val);
  analogWrite(G_Pin, pwm_val);
  analogWrite(B_Pin, pwm_val);
}

void setup() {
  
  Serial.begin(bauds);
  
  //Filter coeffs assign
  low_filter_def.setFilterCoeffs(low_filter);
  mid_filter_def.setFilterCoeffs(mid_filter);
  main_filter_def.setFilterCoeffs(main_filter);
  
  sequence = linear; //Starting sequence
}

void loop() {
  
  switch(sequence){
    case audio:
      audioSequence();
      linear_start_done=false;
    break;
    
    case sine:
      sineSequence(true, true, true);
    break;

    case sine_phaser:
      sinePhaser();
    break;
    
    case strobe:
      strobeSequence();
    break;
    
    case linear:
      linearSequence();
    break;

    default:
    break;
  }
}