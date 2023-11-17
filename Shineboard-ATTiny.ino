// ATTiny Shineboard
#include <EEPROM.h>
#include <TinyWireM.h>
#include <Arduino.h>
#include <math.h>
//#include <Tiny4kOLED.h>


#define UxAddr       0
#define UyAddr       2
#define ULxAddr      4
#define ULyAddr      6
#define LxAddr       8
#define LyAddr       10
#define DLxAddr      12
#define DLyAddr      14
#define DxAddr       16
#define DyAddr       18
#define DRxAddr      20
#define DRyAddr      22
#define RxAddr       24
#define RyAddr       26
#define URxAddr      28
#define URyAddr      30
#define CxAddr       32
#define CyAddr       32
#define dacAddr      0x0//TODO, mapped by hardware

#define cmdDacOut1   0x0
#define cmdDacOut2   0x1//TODO, check data sheets
#define outputVoltXMin  0
#define outputVoltXMax  4096
#define outputVoltYMin  0
#define outputVoltYMax  4096

#define MAX_ORDER    20
#ifndef CURVE_FIT_DEBUG
#define CURVE_FIT_DEBUG 0
#endif

/* Enum for error messages */
enum curveFitERROR{
	ORDER_AND_NCOEFFS_DO_NOT_MATCH = -1,
	ORDER_INCORRECT = -2,
	NPOINTS_INCORRECT = -3
};

//resitor ladder button mux for ADC input- Voltage must be above ~3V to not cause reset pin to activate
//10kohm res 3.090v/button adc - 10bit ADC of 3.3v ~ 959/1024  Ranges 955-962 adc
//15kohm res 3.157v/button adc - 10bit ADC of 3.3v ~ 980/1024  Ranges 976-983 adc
//20kohm res 3.191v/button adc - 10bit ADC of 3.3v ~ 990/1024  Ranges 987-993 adc
#define b1Lower      955
#define b1Upper      962
#define b2Lower      976
#define b2Upper      983
#define b3Lower      987
#define b3Upper      993

struct mapping{
   int Ux;
   int ULx;
   int Lx;
   int DLx;
   int Dx;
   int DRx;
   int Rx;
   int URx;
   int Cx;
   int Uy;
   int ULy;
   int Ly;
   int DLy;
   int Dy;
   int DRy;
   int Ry;
   int URy;
   int Cy;
};

mapping notch;
unsigned int outputValueX = 0;
unsigned int outputValueY = 0;
unsigned int linearizedValueX = 0;
unsigned int linearizedValueY = 0;
unsigned int mappedValueX = 0;
unsigned int mappedValueY = 0;
const int adcXPin = A2;  
const int adcYPin = A3;
const int adcButtonPin = A0;
const int ledPin = 6;
int xpower = 3;
int order = 3;
double coeffsX[4]; //Hardcoded now, CurveFitting has them as xpower + 1
double coeffsY[4]; 

//SSOLED ssoled;

void setup() {

  pinMode(adcXPin, INPUT);
  pinMode(adcYPin, INPUT);
  pinMode(6, OUTPUT);
  TinyWireM.begin();

  notch.Ux = EEPROM_read(UxAddr);
  notch.Uy = EEPROM_read(UyAddr);
  notch.ULx = EEPROM_read(ULxAddr);
  notch.ULy = EEPROM_read(ULyAddr);
  notch.Lx = EEPROM_read(LxAddr);
  notch.Ly = EEPROM_read(LyAddr);
  notch.DLx = EEPROM_read(DLxAddr);
  notch.DLy = EEPROM_read(DLyAddr);
  notch.Dx = EEPROM_read(DxAddr);
  notch.Dy = EEPROM_read(DyAddr);
  notch.DRx = EEPROM_read(DRxAddr);
  notch.DRy = EEPROM_read(DRyAddr);
  notch.Rx = EEPROM_read(RxAddr);
  notch.Ry = EEPROM_read(RyAddr);
  notch.URx = EEPROM_read(URxAddr);
  notch.URy = EEPROM_read(URyAddr);
  notch.Cx = EEPROM_read(CxAddr);
  notch.Cy = EEPROM_read(CyAddr);
  digitalWrite(6, HIGH);

  double curveInputsx[5] = {notch.Lx, notch.ULx, notch.Cx, notch.URx, notch.Rx};
  double curveInputsy[5] = {notch.Ly, notch.ULy, notch.Cy, notch.URy, notch.Ry};
  double notchConfigs[5] = {-1, -0.71, 0, 0.71, 1};

  
  for (int i = 0; i < sizeof(curveInputsx)/sizeof(double); i++){
    notchConfigs[i] = i;
    curveInputsx[i] = pow(i, xpower);
  }

  //double coeffsX[order+1];
  int retX = fitCurve(order, sizeof(curveInputsx)/sizeof(double), notchConfigs, curveInputsx, sizeof(coeffsX)/sizeof(double), coeffsX);
  

  for (int i = 0; i < sizeof(curveInputsx)/sizeof(double); i++){
    notchConfigs[i] = i;
    curveInputsy[i] = pow(i, xpower);
  }

  //double coeffsY[order+1];
  int retY = fitCurve(order, sizeof(curveInputsy)/sizeof(double), notchConfigs, curveInputsy, sizeof(coeffsY)/sizeof(double), coeffsY);

  
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
  

  outputValueX = analogRead(adcXPin);
  outputValueY = analogRead(adcYPin);

  linearizedValueX = linearize(outputValueX, coeffsX);
  linearizedValueY = linearize(outputValueY, coeffsY);

  mappedValueX = map(linearizedValueX, notch.Lx, notch.Rx, outputVoltXMin, outputVoltXMax);
  mappedValueY = map(linearizedValueY, notch.Dy, notch.Uy, outputVoltYMin, outputVoltYMax);

  //String adcString = String("X:" + String(outputValueX) + ", Y:" + String(outputValueY) +"  ");
  //oledWriteString(&ssoled, 0,0,1,(char *)c_str("X:" + String(outputValueX) + ", Y:" + String(outputValueY) +"  "), FONT_SMALL, 1, 1);
  //oledWriteString(&ssoled, 0,0,1,(char *)"Written by Larry Bank", FONT_SMALL, 1, 1);

  

  writeDAC(cmdDacOut1, mappedValueX);
  writeDAC(cmdDacOut2, mappedValueY);

  if(buttonCheck() != 0x0){ //button held?  config ranges
    if(buttonHeld() == true){
      mapAxisValues();
    }
  }

}


void writeDAC(uint8_t code, uint16_t Data)
{
  uint8_t data_high = Data >> 8;
  uint8_t data_low  = Data & 0xff;
     
  TinyWireM.beginTransmission(dacAddr);
  TinyWireM.send(code);
  TinyWireM.send(data_high);
  TinyWireM.send(data_low);
  TinyWireM.end();
}

char buttonCheck(){
  //ADC button pin
  int adcB = analogRead(adcButtonPin);

  if(adcB >= b1Lower && adcB <= b1Upper){
    return(0x1); //button1
  }

  if(adcB >= b2Lower && adcB <= b2Upper){
    return(0x2); //button2
  }

  if(adcB >= b3Lower && adcB <= b3Upper){
    return(0x3); //button3
  }

  return(0x0); //else no button
}

void EEPROM_write(unsigned char ucAddress, unsigned int ucData){
  byte byte1 = ucData >> 8;
  byte byte2 = ucData & 0xFF;

  EEPROM.write(ucAddress, byte1);
  EEPROM.write(ucAddress+1, byte2);
}

unsigned int EEPROM_read(unsigned char ucAddress){
  byte byte1;
  byte byte2;

  byte1 = EEPROM.read(ucAddress);
  byte2 = EEPROM.read(ucAddress+1);

  return((byte1 << 8) + byte2);
}


bool buttonHeld(){  //test if button is held for ~2 seconds
  digitalWrite(ledPin, LOW);
  for(unsigned char i = 0; i < 250; i++){ // 250 cycles at 8ms delay ~~2seconds
        if (buttonCheck() == 0x0){
          return (false);
        }
        delay(8);  
      }

    digitalWrite(ledPin, HIGH);
    return(true);
}


void mapAxisValues(){

  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.Ux = analogRead(adcXPin);
    notch.Uy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }

  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.ULx = analogRead(adcXPin);
    notch.ULy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }

  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.Lx = analogRead(adcXPin);
    notch.Ly = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }

  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.DLx = analogRead(adcXPin);
    notch.DLy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.Dx = analogRead(adcXPin);
    notch.Dy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.DRx = analogRead(adcXPin);
    notch.DRy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.Rx = analogRead(adcXPin);
    notch.Ry = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.URy = analogRead(adcXPin);
    notch.URy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  while(buttonCheck() != 0x0){}
  delay(100); //debounce

  while(buttonCheck() == 0x0){
    digitalWrite(ledPin, LOW);
    notch.Cx = analogRead(adcXPin);
    notch.Cy = analogRead(adcYPin);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
  
  digitalWrite(ledPin, HIGH);

  //write collected ADC values to EEPROM
  EEPROM_write(UxAddr, notch.Ux);
  EEPROM_write(UyAddr, notch.Uy);
  EEPROM_write(ULxAddr, notch.ULx);
  EEPROM_write(ULyAddr, notch.ULy);
  EEPROM_write(LxAddr, notch.Lx);
  EEPROM_write(LyAddr, notch.Ly);
  EEPROM_write(DLxAddr, notch.DLx );
  EEPROM_write(DLyAddr, notch.DLy);
  EEPROM_write(DxAddr, notch.Dx);
  EEPROM_write(DyAddr, notch.Dy);
  EEPROM_write(DRxAddr, notch.DRx);
  EEPROM_write(DRyAddr, notch.DRy);
  EEPROM_write(RxAddr, notch.Rx);
  EEPROM_write(RyAddr, notch.Ry);
  EEPROM_write(URxAddr, notch.URx);
  EEPROM_write(URyAddr, notch.URy);
  EEPROM_write(CxAddr, notch.Cx);
  EEPROM_write(CyAddr, notch.Cy);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PhobGCC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float linearize(const float point, double coefficients[]){
	return (coefficients[0]*(point*point*point) + coefficients[1]*(point*point) + coefficients[2]*point + coefficients[3]);
};


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CURVE FITTING (repurposed for ATTiny  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                                                                serial prints removed)

/*void printMat(const char *s, double*m, int n){
  Streantln(s);
  char buf[40];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      snprintf(buf, 40, "%30.4f\t", m[i*n+j]);
      Serial.print(buf);
    }
    Serial.println();
  }
}

void showmat(const char *s, double **m, int n){
  Stream.println(s);
  char buf[40];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++){
      snprintf(buf, 40, "%30.4f\t", m[i][j]);
      Serial.print(buf);
    }
    Serial.println();
  }
}*/

void cpyArray(double *src, double*dest, int n){
  for (int i = 0; i < n*n; i++){
    dest[i] = src[i];
  }
}

void subCol(double *mat, double* sub, uint8_t coln, uint8_t n){
  if (coln >= n) return;
  for (int i = 0; i < n; i++){
    mat[(i*n)+coln] = sub[i];
  }
}

/*Determinant algorithm taken from https://codeforwin.org/2015/08/c-program-to-find-determinant-of-matrix.html */
int trianglize(double **m, int n)
{
  int sign = 1;
  for (int i = 0; i < n; i++) {
    int max = 0;
    for (int row = i; row < n; row++)
      if (fabs(m[row][i]) > fabs(m[max][i]))
        max = row;
    if (max) {
      sign = -sign;
      double *tmp = m[i];
      m[i] = m[max], m[max] = tmp;
    }
    if (!m[i][i]) return 0;
    for (int row = i + 1; row < n; row++) {
      double r = m[row][i] / m[i][i];
      if (!r) continue;
      for (int col = i; col < n; col ++)
        m[row][col] -= m[i][col] * r;
    }
  }
  return sign;
}
 
double det(double *in, int n, uint8_t prnt)
{
  double *m[n];
  m[0] = in;
 
  for (int i = 1; i < n; i++)
    m[i] = m[i - 1] + n;
  //if(prnt) showmat("Matrix", m, n);
  int sign = trianglize(m, n);
  if (!sign)
    return 0;
  //if(prnt) showmat("Upper triangle", m, n);
  double p = 1;
  for (int i = 0; i < n; i++)
    p *= m[i][i];
  return p * sign;
}
/*End of Determinant algorithm*/

//Raise x to power
double curveFitPower(double base, int exponent){
  if (exponent == 0){
    return 1;
  } else {
    double val = base;
    for (int i = 1; i < exponent; i++){
      val = val * base;
    }
    return val;
  }
}

int fitCurve (int order, int nPoints, double py[], int nCoeffs, double *coeffs) {
  uint8_t maxOrder = MAX_ORDER;
  if (nCoeffs != order + 1) return ORDER_AND_NCOEFFS_DO_NOT_MATCH; 	// no of coefficients is one larger than the order of the equation
  if (nCoeffs > maxOrder || nCoeffs < 2) return ORDER_INCORRECT; 		//matrix memory hard coded for max of 20 order, which is huge
  if (nPoints < 1) return NPOINTS_INCORRECT; 							//Npoints needs to be positive and nonzero
  int i, j;
  double T[MAX_ORDER] = {0}; //Values to generate RHS of linear equation
  double S[MAX_ORDER*2+1] = {0}; //Values for LHS and RHS of linear equation
  double denom; //denominator for Cramer's rule, determinant of LHS linear equation
  double x, y;
  
  double px[nPoints]; //Generate X values, from 0 to n
  for (i=0; i<nPoints; i++){
	px[i] = i;
  }
  
  for (i=0; i<nPoints; i++) {//Generate matrix elements
    x = px[i];
    y = py[i];
    for (j = 0; j < (nCoeffs*2)-1; j++){
      S[j] += curveFitPower(x, j); // x^j iterated , S10 S20 S30 etc, x^0, x^1...
    }
    for (j = 0; j < nCoeffs; j++){
      T[j] += y * curveFitPower(x, j); //y * x^j iterated, S01 S11 S21 etc, x^0*y, x^1*y, x^2*y...
    }
  }

  double masterMat[nCoeffs*nCoeffs]; //Master matrix LHS of linear equation
  for (i = 0; i < nCoeffs ;i++){//index by matrix row each time
    for (j = 0; j < nCoeffs; j++){//index within each row
      masterMat[i*nCoeffs+j] = S[i+j];
    }
  }
  
  double mat[nCoeffs*nCoeffs]; //Temp matrix as det() method alters the matrix given
  cpyArray(masterMat, mat, nCoeffs);
  denom = det(mat, nCoeffs, CURVE_FIT_DEBUG);
  cpyArray(masterMat, mat, nCoeffs);

  //Generate cramers rule mats
  for (i = 0; i < nCoeffs; i++){ //Temporary matrix to substitute RHS of linear equation as per Cramer's rule
    subCol(mat, T, i, nCoeffs);
    coeffs[nCoeffs-i-1] = det(mat, nCoeffs, CURVE_FIT_DEBUG)/denom; //Coefficients are det(M_i)/det(Master)
    cpyArray(masterMat, mat, nCoeffs);
  }
  return 0;
}

int fitCurve (int order, int nPoints, double px[], double py[], int nCoeffs, double *coeffs) {
  uint8_t maxOrder = MAX_ORDER;
  if (nCoeffs != order + 1) return ORDER_AND_NCOEFFS_DO_NOT_MATCH; 	//Number of coefficients is one larger than the order of the equation
  if(nCoeffs > maxOrder || nCoeffs < 2) return ORDER_INCORRECT; 		//Matrix memory hard coded for max of 20 order, which is huge
  if (nPoints < 1) return NPOINTS_INCORRECT; 							//Npoints needs to be positive and nonzero
  int i, j;
  double T[MAX_ORDER] = {0}; //Values to generate RHS of linear equation
  double S[MAX_ORDER*2+1] = {0}; //Values for LHS and RHS of linear equation
  double denom; //denominator for Cramer's rule, determinant of LHS linear equation
  double x, y;
  
  for (i=0; i<nPoints; i++) {//Generate matrix elements
    x = px[i];
    y = py[i];
    for (j = 0; j < (nCoeffs*2)-1; j++){
      S[j] += curveFitPower(x, j); // x^j iterated , S10 S20 S30 etc, x^0, x^1...
    }
    for (j = 0; j < nCoeffs; j++){
      T[j] += y * curveFitPower(x, j); //y * x^j iterated, S01 S11 S21 etc, x^0*y, x^1*y, x^2*y...
    }
  }

  double masterMat[nCoeffs*nCoeffs]; //Master matrix LHS of linear equation
  for (i = 0; i < nCoeffs ;i++){//index by matrix row each time
    for (j = 0; j < nCoeffs; j++){//index within each row
      masterMat[i*nCoeffs+j] = S[i+j];
    }
  }
  
  double mat[nCoeffs*nCoeffs]; //Temp matrix as det() method alters the matrix given
  cpyArray(masterMat, mat, nCoeffs);
  denom = det(mat, nCoeffs, CURVE_FIT_DEBUG);
  cpyArray(masterMat, mat, nCoeffs);

  //Generate cramers rule mats
  for (i = 0; i < nCoeffs; i++){ //Temporary matrix to substitute RHS of linear equation as per Cramer's rule
    subCol(mat, T, i, nCoeffs);
    coeffs[nCoeffs-i-1] = det(mat, nCoeffs, CURVE_FIT_DEBUG)/denom; //Coefficients are det(M_i)/det(Master)
    cpyArray(masterMat, mat, nCoeffs);
  }
  return 0;
}
