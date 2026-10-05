/*
 * MotorDriver.C
 *
 *  Created on: 5 de out. de 2026
 *      Author: David Amorim Silva
 */

#include <MotorDriver.h>



void SetInverter(ThreePhaseInverter* Inverter,  TIM_HandleTypeDef *TimersABC[], uint32_t ChannelABC[], GPIO_TypeDef *EnablePorts[], uint16_t EnablePins[]){
	for (i=0;i<3;i++){
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
