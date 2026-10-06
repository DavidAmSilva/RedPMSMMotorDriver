/*
 * MotorDriver.C
 *
 *  Created on: 5 de out. de 2026
 *      Author: David Amorim Silva
 */

#include <MotorDriver.h>

void SetInverter(ThreePhaseInverter* Inverter,  TIM_HandleTypeDef *TimersABC[], uint32_t ChannelABC[], GPIO_TypeDef *EnablePorts[], uint16_t EnablePins[]){
	for (uint8_t i=0;i<3;i++){
		Inverter->INChannel[i] = TimersABC[i];
		Inverter->INChannel[i] = ChannelABC[i];
		Inverter->ENPort[i] = EnablePorts[i];
		Inverter->ENPin[i] = EnablePins[i];
	};
};

void SetCurrentSensor (Current_sensor *Sensor, ADC_HandleTypeDef *ADCHandler, uint32_t ADCChannel ){
	Sensor->ADCHandler = ADCHandler;
	Sensor->channel = ADCChannel;
};

void SetMotorParameters(Motor *PMSM , float L, float Phy, float Telmax, float Imax, uint8_t PolePairs){
	PMSM->L = L;
	PMSM->Phy_nom = Phy;
	PMSM->TelMax = Telmax;
	PMSM->Imax = Imax;
	PMSM->PP=PolePairs;
	PMSM->ControllerFlag = 0;
};

void SetMotorFOC(Motor *PMSM, float ElectricalController[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB){
	for(uint8_t i=0;i<2;i++){
		PMSM->Electrical_controller[i] =ElectricalController[i];
		PMSM->Mechanical_controller[i] =MechanicalController[i];
	};
	PMSM->PhaseA = PhaseA;
	PMSM->PhaseB = PhaseB;
	PMSM->id=0;
	PMSM->iq=0;
	PMSM->ControllerFlag = 2;
};

void SetMotorClassicalDTC(Motor *PMSM, float HyteresysBands[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB){
	for(uint8_t i=0;i<2;i++){
		PMSM->HysteresysBands[i] =HyteresysBands[i];
		PMSM->Mechanical_controller[i] =MechanicalController[i];
	};
	PMSM->PhaseA = PhaseA;
	PMSM->PhaseB = PhaseB;
	PMSM->ialfa=0;
	PMSM->ibeta=0;
	PMSM->ControllerFlag = 3;
};

void SetMotorSixStep(Motor *PMSM, float MechanicalController[]){
	for (uint8_t i=0;i<2;i++){
		PMSM->Mechanical_controller[i] = MechanicalController[i];
	};
	PMSM->ControllerFlag = 1;
}



