/*
 * MotorDriver.h
 *
 *  Created on: 5 de out. de 2026
 *      Author: David Amorim Silva
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_

#include "gpio.h"
#include "tim.h"
#include "stm32g4xx_hal.h"
#include "arm_math.h"

#define CALIBRATION_SAMPLES 1024
#define TWOPI 6.2831853f
#define SQRT3 1.73205f
#define ALIGNSTEPS 20

typedef struct {

	TIM_HandleTypeDef *INTimer[3];
	uint32_t INChannel[3];
	GPIO_TypeDef *ENPort[3];
	uint16_t ENPin[3];
	uint8_t dutyMode;

	ADC_HandleTypeDef *Busvoltage_ADCHandler;
	uint32_t BusvoltageADC_channel;
	uint8_t BusVoltageADC_flag;
	float Bus_voltage_converter_gain; //Ganho do divisor resistivo (V/V)
	float Bus_voltage;

} ThreePhaseInverter;

typedef struct {
	ADC_HandleTypeDef *ADCHandler;
	uint32_t channel;
	float currentGain;
	uint32_t Offset;
} Current_sensor;

typedef struct {
	TIM_HandleTypeDef *Timer;
	uint16_t PPR; //Pulsos por revolução do encoder conectado
	volatile float MechanicalAngle;
	volatile float ElectricalAngle;
	uint32_t CPR; //Contagens por revolução com encoder mode
	int16_t MechanicalCounter;
}Encoder;


typedef struct {
	float L, Phy_nom; // Indutância do motor (Considerando apenas indutancia total em alphabeta), Fluxo nominal do motor e Constante de velocidade do motor
	uint8_t PP; //Pares de polos
	float Kt;
	float TelMax, Imax; // Parâmetros para limitar torque e corrente do motor
	volatile float ialfa, ibeta, id, iq; // Corrente em alfa e beta, d e q

	//Inversor trifásico associado ao motor
	ThreePhaseInverter *Inverter;

	// Declaração do encoder
	Encoder *encoder;
	volatile uint8_t sector;
	volatile float AngularSpeed;

	// Declaração dos sensores de corrente da placa
	Current_sensor *PhaseA;
	Current_sensor *PhaseB;

	//Controladores /
	uint32_t MechTs;
	uint32_t elecTs;
	uint32_t MechanicalTsRatio;
	float Mechanical_controller[2];
	float Electrical_controller[2];
	float HysteresysBands[2];
	volatile uint32_t controller_counter;

	uint8_t ControllerFlag; // 0 = nao config | 1 = SixStep | 2 = FOC | 3 = DTC Classico

} Motor;

//Funções para Inversor de frequência
void SetInverter(ThreePhaseInverter* Inverter,  TIM_HandleTypeDef *TimersABC[], uint32_t ChannelABC[], GPIO_TypeDef *EnablePorts[], uint16_t EnablePins[], uint8_t PWMMode);
void SetInverter_BusVoltage();
void SetInverter_BusVoltageADC();
void Inverter_updateBusVoltage();
void SetPWM(TIM_HandleTypeDef *Timer, uint32_t Channel, float dutyCycle);
void setEncoder(Encoder *encoder, TIM_HandleTypeDef *Timer,uint16_t PPR );

void SetCurrentSensor(Current_sensor *Sensor, ADC_HandleTypeDef *ADCHandler, uint32_t ADCChannel, float CurrentGain  );
void SetMotorParameters(Motor *PMSM , float L, float Phy, float Kt,float Telmax, float Imax, uint8_t PolePairs, Encoder *encoder);
void SetMotorFOC(Motor *PMSM, float ElectricalController[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB ,uint32_t ElectricalTs, uint32_t MechanicalTs);
void SetMotorClassicalDTC(Motor *PMSM, float HyteresysBands[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB,uint32_t ElectricalTs, uint32_t MechanicalTs);
void SetMotorSixStep(Motor *PMSM, float MechanicalController[], uint32_t MechanicalTs,uint32_t InterruptTs);


void SVPWM(float Va, float Vb, float Vc, float Vbus ,float dutyCycles[]);

//Funcoes voltadas para alinhamento do rotor
void ApplyVoltageVector (Motor *PMSM, float Vd, float Vq, float Vbus, float angleRad);
void AlignMotor (Motor *PMSM, float Valign);

//Encoder
void updateEncoder(Motor *PMSM);


//Sensor de corrente
uint32_t ReadADC_Channel(ADC_HandleTypeDef *hadc, uint32_t channel);
void CalibrateCurrentOffset(Motor *PMSM);
void ReadMotorCurrents(Motor *PMSM, float *I_a, float *I_b);

//FuncoesControle
void updateFOCTorqueController(Motor *PMSM, float Telref);
void updateFOCSpeedController(Motor *PMSM, float ref);




#endif /* INC_MOTORDRIVER_H_ */
