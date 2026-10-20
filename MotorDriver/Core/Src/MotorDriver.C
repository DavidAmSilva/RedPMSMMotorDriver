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

void setEncoder(Encoder *encoder, TIM_HandleTypeDef Timer,uint16_t PPR ){
    encoder->Timer = Timer;
    encoder->PPR = PPR;
    encoder->CPR = (uint32_t)PPR * 4; // O Modo Encoder do STM32 (TI12) conta nas bordas de subida e descida de A e B
    encoder->MechanicalAngle = 0.0f;
    encoder->MechanicalCounter = 0;

    __HAL_TIM_SET_COUNTER(Timer, 0);

    // Inicializa o periférico de hardware do Timer em Encoder Mode
    HAL_TIM_Encoder_Start(Timer, TIM_CHANNEL_ALL);
}

void SetCurrentSensor (Current_sensor *Sensor, ADC_HandleTypeDef *ADCHandler, uint32_t ADCChannel, float CurrentGain ){
	Sensor->ADCHandler = ADCHandler;
	Sensor->channel = ADCChannel;
	Sensor->currentGain = CurrentGain;
}

void SetMotorParameters(Motor *PMSM , float L, float Phy, float Telmax, float Imax, uint8_t PolePairs, Encoder *encoder){
	PMSM->L = L;
	PMSM->Phy_nom = Phy;
	PMSM->TelMax = Telmax;
	PMSM->Imax = Imax;
	PMSM->PP=PolePairs;
	PMSM->ControllerFlag = 0;
	PMSM->AngularSpeed = 0;
	PMSM->encoder = encoder;
}

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
}

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
}

void SetMotorSixStep(Motor *PMSM, float MechanicalController[], uint32_t MechanicalTs, uint32_t InterruptTs){
	for (uint8_t i=0;i<2;i++){
		PMSM->Mechanical_controller[i] = MechanicalController[i];
	};
	PMSM->ControllerFlag = 1;
	PMSM->MechanicalTsRatio = MechanicalTs/InterruptTs;
}



void SVPWM(float Va, float Vb, float Vc, float Vbus ,float dutyCycles[]){
	Va=(Va/Vbus)*SQRT3;
	Vb=(Vb/Vbus)*SQRT3;
	Vc=(Vc/Vbus)*SQRT3;
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

	dutyCycles[0] = 0.5f*(va_mod+1);
	dutyCycles[1] = 0.5f*(vb_mod+1);
	dutyCycles[2] = 0.5f*(vc_mod+1);

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
	float dutyCycles[3] = {0,0,0};
	SVPWM(Va,Vb,Vc,Vbus,dutyCycles);
	for(uint8_t i=0;i<3;i++){
		HAL_GPIO_WritePin(PMSM->Inverter->ENPort[i], PMSM->Inverter->ENPin[i], GPIO_PIN_SET); // Passar função para "enable" de FOC
		SetPWM(PMSM->Inverter->INTimer[i], PMSM->Inverter->INChannel[i], dutyCycles[i]);
	}

}

void AlignMotor (Motor *PMSM, float Valign){
	for (uint16_t i = 0;i<ALIGNSTEPS;i++){
		ApplyVoltageVector(PMSM, Valign , 0, PMSM->Inverter->Bus_voltage, i*2*PI/ALIGNSTEPS);
		HAL_Delay(100);
	}
	for (uint16_t i=ALIGNSTEPS;i>0;i--){
		ApplyVoltageVector(PMSM, Valign , 0, PMSM->Inverter->Bus_voltage, i*2*PI/ALIGNSTEPS);
		HAL_Delay(100);
	}
	PMSM->position = 0;
}


uint32_t ReadADC_Channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configuração do canal desejado para amostragem regular
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;      // Primeiro lugar na sequência regular
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; // Tempo de amostragem adequado para estabilidade AVALIAR TEMPO DE AMOSTRAGEM PARA SISTEMA
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;         // Modo de entrada simples (Single-ended)
    sConfig.OffsetNumber = ADC_OFFSET_NONE;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0; // Falha na configuração do canal
    }

    // Dispara a conversão no canal recém-configurado
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return val;
    }

    HAL_ADC_Stop(hadc);
    return 0;
}



void CalibrateCurrentOffset(Motor *PMSM){
	uint32_t offset = 0;
	//Desativando todas as saídas que possam gerar corrente
	for (uint8_t i =0;i<3;i++){
		SetPWM(PMSM->Inverter->INTimer[i], PMSM->Inverter->INChannel[i], 0); //Desativando PWM
		HAL_GPIO_WritePin(PMSM->Inverter->ENPort[i], PMSM->Inverter->ENPin[i], GPIO_PIN_RESET); //Desativando Enables
	}
	HAL_Delay(10);

	uint32_t Phase_offset[2] = {0,0};
	for (uint8_t i =0 ;i<2;i++){
		Phase_offset[i] = 0;
		for (uint16_t s = 0;s<CALIBRATION_SAMPLES;s++){
			if(i == 0){
				Phase_offset[i] += ReadADC_Channel(PMSM->PhaseA->ADCHandler, PMSM->PhaseA->channel);
			}	else {
				Phase_offset[i] += ReadADC_Channel(PMSM->PhaseB->ADCHandler, PMSM->PhaseB->channel);
			}

		}
		Phase_offset[i] = Phase_offset[i]/CALIBRATION_SAMPLES;
		if (i ==0){
			PMSM->PhaseA->Offset = Phase_offset[i];
		} else {
			PMSM->PhaseB->Offset = Phase_offset[i];
		}
	}
}


void ReadMotorCurrents(Motor *PMSM, float *I_a, float *I_b){
	*I_a = (ReadADC_Channel(PMSM->PhaseA->ADCHandler,PMSM->PhaseA->channel) - PMSM->PhaseA->Offset)*PMSM->PhaseA->currentGain;
	*I_b = (ReadADC_Channel(PMSM->PhaseB->ADCHandler,PMSM->PhaseB->channel) - PMSM->PhaseB->Offset)*PMSM->PhaseB->currentGain;

}

void updateEncoder(Motor *PMSM){
	int16_t count = __HAL_TIM_GetCounter(PMSM->encoder->TIMER);
	float rawAngle = PMSM->encoder->MechanicalAngle + cout*CPR/(TWOPI);
	if (rawAngle > TWOPI){
		 rawAngle-= TWOPI;
	} else if (rawAngle<0){
		rawAngle += TWOPI;
	}
	PMSM->encoder->MechanicalAngle = rawAngle;
	rawAngle = PMSM->encoder->ElectricalAngle + (PMSM->PP)*PMSM->encoder->MechanicalAngle;
	if (rawAngle > TWOPI){
		 rawAngle-= TWOPI;
	} else if (rawAngle<0){
		rawAngle += TWOPI;
	}
	PMSM->encoder->ElectricalAngle = rawAngle;
	PMSM->encoder->MechanicalCounter += count; //Este valor irá acumulando até a interrupção mecanica ser disparada
}

