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



typedef struct {

	TIM_HandleTypeDef *INTimer[3];
	uint32_t INChannel[3];
	GPIO_TypeDef *ENPort[3];
	uint16_t ENPin[3];

} ThreePhaseInverter;

typedef struct {
	ADC_HandleTypeDef *ADCHandler;
	uint32_t channel;
} Current_sensor;


typedef struct {
	float L, Phy_nom; // Indutância do motor (Considerando apenas indutancia total em alpha), Fluxo nominal do motor e Constante de velocidade do motor
	uint8_t PP; //Pares de polos
	float TelMax, Imax; // Parâmetros para limitar torque e corrente do motor
	volatile float ialfa, ibeta, id, iq; // Corrente em alfa e beta, d e q
	float Mechanical_controller[2];
	float Electrical_controller[2];
	float HysteresysBands[2]; //Banda de histerese de Torque, Banda de histerese de fluxo;
	volatile uint32_t position; //Posição em inteiros, decidir escala
	volatile uint8_t sector;

	// Declaração dos sensores de corrente da placa
	Current_sensor *PhaseA;
	Current_sensor *PhaseB;

	//Inversor trifásico associado ao motor
	ThreePhaseInverter *Inverter;

	uint8_t ControllerFlag; // 0 = nao config | 1 = SixStep | 2 = FOC | 3 = DTC Classico

} Motor;

void SetInverter(ThreePhaseInverter* Inverter,  TIM_HandleTypeDef *TimersABC[], uint32_t ChannelABC[], GPIO_TypeDef *EnablePorts[], uint16_t EnablePins[]);
void SetCurrentSensor(Current_sensor *Sensor, ADC_HandleTypeDef *ADCHandler, uint32_t ADCChannel );
void SetMotorParameters(Motor *PMSM , float L, float Phy, float Telmax, float Imax, uint8_t PolePairs);
void SetMotorFOC(Motor *PMSM, float ElectricalController[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB);
void SetMotorClassicalDTC(Motor *PMSM, float HyteresysBands[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB);
void SetMotorSixStep(Motor *PMSM, float MechanicalController[]);
#endif /* INC_MOTORDRIVER_H_ */
