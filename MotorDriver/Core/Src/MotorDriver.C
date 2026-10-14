/*
 * MotorDriver.C
 *
 *  Created on: 5 de out. de 2026
 *      Author: David Amorim Silva
 */

#include <MotorDriver.h>

void SetInverter(ThreePhaseInverter* Inverter,  TIM_HandleTypeDef *TimersABC[], uint32_t ChannelABC[], GPIO_TypeDef *EnablePorts[], uint16_t EnablePins[], uint8_t PWMMode){
	for (uint8_t i=0;i<3;i++){
		Inverter->INTimer[i] = TimersABC[i];
		Inverter->INChannel[i] = ChannelABC[i];
		Inverter->ENPort[i] = EnablePorts[i];
		Inverter->ENPin[i] = EnablePins[i];
	};
	Inverter->BusVoltageADC_flag = 0;
	Inverter->dutyMode = PWMMode;
};

void SetPWM(TIM_HandleTypeDef *Timer, uint32_t Channel, float dutyCycle){
	uint32_t CCR = 0;
	uint32_t ARR = Timer->Instance->ARR;
	if(dutyCycle<0) dutyCycle = 0;

	if(dutyCycle<1){
		CCR = ARR*dutyCycle;
	} else{
		CCR = ARR;
	}

	__HAL_TIM_SET_COMPARE(Timer, Channel, CCR);
}


void SetCurrentSensor (Current_sensor *Sensor, ADC_HandleTypeDef *ADCHandler, uint32_t ADCChannel, float CurrentGain ){
	Sensor->ADCHandler = ADCHandler;
	Sensor->channel = ADCChannel;
	Sensor->currentGain = CurrentGain;
};

void SetMotorParameters(Motor *PMSM , float L, float Phy, float Telmax, float Imax, uint8_t PolePairs){
	PMSM->L = L;
	PMSM->Phy_nom = Phy;
	PMSM->TelMax = Telmax;
	PMSM->Imax = Imax;
	PMSM->PP=PolePairs;
	PMSM->ControllerFlag = 0;
	PMSM->AngularSpeed = 0;
};

void SetMotorFOC(Motor *PMSM, float ElectricalController[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB ,uint32_t ElectricalTs, uint32_t MechanicalTs){
	for(uint8_t i=0;i<2;i++){
		PMSM->Electrical_controller[i] =ElectricalController[i];
		PMSM->Mechanical_controller[i] =MechanicalController[i];
	};
	PMSM->PhaseA = PhaseA;
	PMSM->PhaseB = PhaseB;
	PMSM->id=0;
	PMSM->iq=0;
	PMSM->ControllerFlag = 2;
	PMSM->MechanicalTsRatio = MechanicalTs/ElectricalTs;
};

void SetMotorClassicalDTC(Motor *PMSM, float HyteresysBands[],float MechanicalController[], Current_sensor *PhaseA, Current_sensor *PhaseB,uint32_t ElectricalTs, uint32_t MechanicalTs){
	for(uint8_t i=0;i<2;i++){
		PMSM->HysteresysBands[i] =HyteresysBands[i];
		PMSM->Mechanical_controller[i] =MechanicalController[i];
	};
	PMSM->PhaseA = PhaseA;
	PMSM->PhaseB = PhaseB;
	PMSM->ialfa=0;
	PMSM->ibeta=0;
	PMSM->ControllerFlag = 3;
	PMSM->MechanicalTsRatio = MechanicalTs/ElectricalTs;
};

void SetMotorSixStep(Motor *PMSM, float MechanicalController[], uint32_t MechanicalTs, uint32_t InterruptTs){
	for (uint8_t i=0;i<2;i++){
		PMSM->Mechanical_controller[i] = MechanicalController[i];
	};
	PMSM->ControllerFlag = 1;
	PMSM->MechanicalTsRatio = MechanicalTs/InterruptTs;
}



void SVPWM(float Va, float Vb, float Vc, float dutyCycles[]){
	float Vmax = Va;
	if(Vb> Vmax) Vmax = Vb;
	if(Vc>Vmax) Vmax = Vc;

	float Vmin = Va;
	if(Vb< Vmin) Vmin = Vb;
	if(Vc< Vmin) Vmin = Vc;

	float V_offset = -0.5f*(Vmax + Vmin);

	float va_mod = Va + V_offset;
	float vb_mod = Vb + V_offset;
	float vc_mod = Vc + V_offset;

	dutyCycles[0] = 0.5*(va_mod+1);
	dutyCycles[1] = 0.5*(vb_mod+1);
	dutyCycles[2] = 0.5*(vc_mod+1);

}

void ApplyVoltageVector (Motor *PMSM, float Vd, float Vq, float Vbus, float angleRad){
	float Valfa = 0;
	float Vbeta = 0;
	float Va=0;
	float Vb =0;
	float Vc =0;
	float SinCos[2] = {0,0};
	SinCos[0] = arm_sin_f32(angleRad);
	SinCos[1] = arm_cos_f32(angleRad);
	arm_inv_park_f32(Vd, Vq, &Valfa, &Vbeta, SinCos[0], SinCos[1]);
	arm_inv_clarke_f32(Valfa, Vbeta, &Va, &Vb);
	Vc = -1*(Va+Vb);
	Va=(Va/Vbus)*1.735;
	Vb=(Vb/Vbus)*1.735;
	Vc=(Vc/Vbus)*1.735;

	float dutyCycles[3] = {0,0,0};
	SVPWM(Va,Vb,Vc,dutyCycles);
	for(uint8_t i=0;i<3;i++){
		SetPWM(PMSM->Inverter->INTimer[i], PMSM->Inverter->INChannel[i], dutyCycles[i]);
	}

}



