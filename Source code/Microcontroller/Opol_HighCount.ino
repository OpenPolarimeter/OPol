/*
MIT License
Copyright (c) 2020 Andrew Harvie
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ADC.h>
#include "sliding_dft_extended.h"
#define ledPin 13
#define pi 3.141592 //something to do with circles
#define numBins 100 //only need first few bins (motor is pretty slow)
#include <elapsedMillis.h>

//Pin for light-frequency converter input
#define ltfPin 23

//////////////////////
/*CHANGE TO true IF YOU'RE USING THE LIGHT-FREQUENCY CONVERTER! */

bool ltf = false;
/////////////////////

//Variables
//Timing
elapsedMicros timeElapsed;
unsigned int looptime = 1000; //in micros
bool wait = true;

//ltfc stuf
volatile int count = 0;
int lastcount = 0;
int grabcount = 0;
float ref_value;


//Calculation
float corr[numBins];
float bigCorr = 0;
int peakLocation;
float phase0;
float phase1;
float phaseGap;

//output
int sampleCheck = 10; //number of samples to wait between comparisons
int stabCheckCount = 0;
int initiating = 0;
float freqSpace;
float lambda = 0.02;
float min_lambda = 1 - lambda;
float alpha = 0.01;
float min_alpha = 1 - alpha;
bool first = true;
float filtered;

//Interpolation
float peakLeft;
float peakCentre;
float peakRight;
float delta;
float preciseFreq;
float gapCentre;
float gapLeft;
float gapRight;


//ADC object
ADC *adc = new ADC();

//Two DFT objects 
static SlidingDFT<float, 8192> dft0;
static SlidingDFT<float, 8192> dft1;

void setup() {
  pinMode(ledPin, OUTPUT); 
  if (ltf == true){
    attachInterrupt(digitalPinToInterrupt(ltfPin), irq1, RISING);
  }
  analogReadRes(12); //set resolution
  Serial.begin(9600);
  digitalWrite(ledPin, HIGH); //on light

}

//ISR for Light-Frequency converter
void irq1(){
  ++count;
}

void loop() {

  //wait between samples
  while(wait == true){
    if (timeElapsed > looptime){
      timeElapsed = 0;
      wait = false;
      break;      
    } 
  }

  // read sensors and update sDFTS
  ADC::Sync_result result = adc->analogSyncRead(A1, A2);

  grabcount = count;
  count = 0;

  if (first==true){
    ref_value = (float)grabcount;
  }
  else{
    ref_value = (float)grabcount * alpha + min_alpha*ref_value;
  }
  
  if (ltf == false){
    ref_value = 1.0; //no dividerino
  }

  int voltage0 = result.result_adc0; 
  int voltage1 = result.result_adc1; 

  float value0 = (float)voltage0/ref_value;
  float value1 = (float)voltage1/ref_value;

  dft0.updateLess(value0, numBins);
  dft1.updateLess(value1, numBins);

  //Find common peak by correlating sDFTs
  for(int i=2; i<numBins; ++i){ //first 200 bins
    corr[i] = abs(dft0.dft[i])*abs(dft1.dft[i]);
  }
  
  bigCorr = 0; //peak
  for (int i=2; i<numBins;++i){ //skip first few bins as they're due to DC offset
    if (corr[i] > bigCorr){
      bigCorr = corr[i];
      peakLocation = i;
    }
  }

  //"RMS" values of common DFT peak
  peakLeft = sqrt(corr[peakLocation-1]);
  peakCentre = sqrt(corr[peakLocation]);
  peakRight = sqrt(corr[peakLocation+1]);

  //interpolation of frequency peak

  delta = 2*((peakRight-peakLeft)/(2*peakCentre+peakLeft+peakRight)); //factor of 2 because Hann window


  
 //Linear phase interpolaton
 if(delta > 0.0){ //max to rhs of peak
    gapCentre = arg(dft1.dft[peakLocation])-arg(dft0.dft[peakLocation]);
    gapRight = arg(dft1.dft[peakLocation + 1])-arg(dft0.dft[peakLocation + 1]);

    if (gapCentre < 0.0){
      gapCentre = gapCentre + 2*pi;
    }
    if (gapRight < 0.0){
      gapRight = gapRight + 2*pi;
    }
  
    phaseGap = gapCentre + delta*(gapRight - gapCentre);

 }
 else{ //to LHS of peak
    gapCentre = arg(dft1.dft[peakLocation])-arg(dft0.dft[peakLocation]);
    gapLeft = arg(dft1.dft[peakLocation - 1])-arg(dft0.dft[peakLocation - 1]);

    if (gapCentre < 0.0){
      gapCentre = gapCentre + 2*pi;
    }
    if (gapLeft < 0.0){
      gapLeft = gapLeft + 2*pi;
    }
  
    phaseGap = gapCentre - delta*(gapLeft - gapCentre);

 }
  
  phaseGap = phaseGap*(180/pi);


  ++stabCheckCount;
  if (stabCheckCount == sampleCheck){
    if (first == true){
      filtered = phaseGap;
      first = false;
    }
    else if (initiating < 1025){
      filtered = phaseGap;
      ++initiating;
    }
    else{
      filtered = filtered*min_lambda + lambda*phaseGap;     
    }
//    Serial.print(peakLocation);
//    Serial.print(" ");
    Serial.println(filtered, 7);
    stabCheckCount = 0;
  }

  wait = true; 
    
}
