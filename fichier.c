/*
 * Fichier.c
 *
 *  Created on: Sep 11, 2024
 *      Author: Adm-Radji
 */
#include <fichier.h>

sensor_t sensor =
{
	// ADC1
	1,					//	uint16_t flag_adc_count;
	0,					//	uint16_t flag_adc1;
	{0, 0, 0, 0},		//	uint16_t ADC1_result[4];
	0,					//	uint16_t ADC1_I1_result;
	0,					//	uint16_t flag_adc2;
	0,					//	uint16_t ADC2_I2_result;
	8,					//	uint16_t n_ma;		// nb of elements in moving average, from 1 to 32 max
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},	//	uint16_t i1_ma_buff[32]
	0,					//	uint32_t i1_ma_sum;
	0,					//  uint16_t i1_ma;
	0,					//	uint16_t i1_index;
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},	//	uint16_t i2_ma_buff[32];
	0,					//	uint32_t i2_ma_sum;
	0,					//  uint16_t i2_ma;
	0,					//	uint16_t i2_index;
	// Speed Encoder
	0,			// int flag_encod_sampling;
	0,			// int encoder_last_value;
	0,			// int encoder_last_time10us;
	0,			// int encoder_last_speed;
	0,			// int encoder_actual_value;
	0,			// int encoder_actual_time10us;
	0,			// int encoder_actual_speed (in deci-rpm);
	0,			// int encoder_actual_accel;
	200000,		// int encoder_min_reinit;
	4000000,	// int encoder_max_reinit;
	2000000,	// int encoder_center_reinit;
	1260,		// int encoder_true_speed (in deci-rpm);
	0,			// int encoder_true_speed_min;
	5000,		// int encoder_true_speed_max;
//	2250,		// int encoder_speed_slope_c; - printer MCC calibrated to 100%
	22500,		// int encoder_speed_slope_c; - POLOLU 37D
//	925,		// int encoder_speed_slope_c;
//	61340,		// int encoder_speed_slope_c;
	0,			// int encoder_set_back;
	// Current
	14660, 		// int current_slope_milli;
//	1970,		// int current_intercept1; // IHM07M1
//	1970		// int current_intercept2; // IHM07M1
	2130,		// int current_intercept1; // IHM08M1
	2130		// int current_intercept2; // IHM08M1
};



/* Variables Globales */
int count_ds=0; //Permet de compter le nombre de 10e s
int count_db=0; //Permet de compter le nombre de clignotement
int count_s=0; //Permet de compter les secondes
int flag_display=0; //Permet d'activer l'autorisation d'actualisation du LCD
char buffStr[16];
char buffStr1[14];
char buffStr2[14];
char buffStr3[14];
char buffStr4[14];
char buffStr5[14];
char buffStr6[14];
char buffStr7[14];
int flag_display2=0;
int state_cnc=0;
int flag_bb=0;
int flag_chrono=0;
int alpha=0.5;
int flag_state=0;
int flag_state1=0;
extern int flag_disp_enc;
int display_pwm=0;
int display_cursor=0;
int display_menu=0;
float omega = 0;        // Vitesse angulaire en rad/s
float rpm = 0;          // Vitesse angulaire en RPM
int speed_rpm = 0; // Vitesse en RPM
int fe=10;
int t_previous = 0;    // Lire la valeur du temps instant précédent
int cnt_previous = 0;  // Lire la valeur du compteur instant précédent
int t_actual = 0; // Lire la valeur du temps instant actuel
int cnt_actual = 0; // Lire la valeur du compteur instant précédent
int pulses_per_rev = 360; // Résolution de l'encodeur
int Vm=12; // Tension nominale
int V_consigne=0;
float ke=16/1000;
int i1=0; //Courant à mesurer
int i2=0;
int flag_first=1;
int ref=0;
float sum_asserv=0;
int position_cons=0;
int position_mes=0; //position angulaire en deg
int N_imp_Mp77=2126; //resolution du moteur de TP
int last_error=0;
void Led_Blink()
{
	switch(count_db){
		case 1:
			if (count_ds==0)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
			}
			break;
		case 2:
			if ((count_ds==0) || (count_ds==2))
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
			}
			break;
		case 3:
			if ((count_ds==0) || (count_ds==2) || (count_ds==4))
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
			}
			break;
	}
}

/* Fonction permettant d'afficher la valeur du chrono */
void disp_chrono(char buffStr[16])
{
	sprintf(buffStr, "%4d,%d s  ", count_s,count_ds); //envois la valeur du chrono
	cnc_lcd_i2c_goto(2,10);
	cnc_lcd_i2c_transmit_string(10,buffStr);
}

/* Fonction permettant d'afficher la valeur de l'encoder */
void disp_Encoder(char buffStr[14])
{
	sprintf(buffStr, "%5d     ", TIM8->CNT); //envois la valeur de l'encoder
	cnc_lcd_i2c_goto(3,10);
	cnc_lcd_i2c_transmit_string(10,buffStr);
}

/* Fonction permettant d'afficher la valeur du potentiomètre */
void disp_Potent(char buffStr[14],ADC_HandleTypeDef hadc1)
{
	HAL_ADC_Start_IT(&hadc1); //Start l'ADC
	int adc= 100* HAL_ADC_GetValue(&hadc1)/4033; // conv en pourcentage
	char str[23]="%"; //permet d'afficher le symbole %
	sprintf(buffStr, " %3d %s     ",adc,str);
	cnc_lcd_i2c_goto(4,9);
	cnc_lcd_i2c_transmit_string(10,buffStr);
}

/* Fonction permettant l'initialisation du LCD */
void init_cnc()
{
	//cnc_lcd_i2c_init();

}

void mcc_control()
{
	int T=TIM1->ARR;
	int alpha= HAL_ADC_GetValue(&hadc1)/4033; //On récupère la valeur du potentiomètre qui fera permettra de régler le alpha
	TIM1->CCR1 = alpha*T;
}
void transit()/* Cette fonction de changer l'état de notre machine */
{
	flag_chrono=0;
	if (state_cnc==0){ // Etat de base state=0

		if (flag_bb==1){
				state_cnc=1;
				flag_bb=0;
		}
	}
	if (state_cnc==1){ //Page 2

		/*if (flag_bb==1){
			state_cnc=2;
			//flag_state1=1;
			flag_bb=0;
		}*/


	}
	if (state_cnc==11){ //Arrêt du chronomètre
		/*if (flag_chrono==0){
			state_cnc=111;
		}*/
		if (flag_bb==1)
		{
			state_cnc=20;
			TIM8->CNT=150;
			flag_bb=0;
		}
	}
	/*if (state_cnc==111){ //reset de la page 2
		if (flag_bb==1){
			state_cnc=1;
			flag_bb=0;
		}
	}*/
	if (state_cnc==2){ //Page du moteur à faire tourner
		/*if (flag_bb==1){
			state_cnc=1;
			//cnc_lcd_i2c_transmit_string(,"      ");
			flag_bb=0;
		}*/
	}
	if (state_cnc==4){
		if (flag_bb==1)
		{
			disp_reset();
			state_cnc=44;
			TIM8->CNT=150;
			edit_Alpha();
			set_pwm_control();
			display_pwm=1;
			speed_reg_fixe();
			flag_bb=0;
		}
	}
	if (state_cnc==5){
		if (flag_bb==1)
		{
			disp_reset();
			state_cnc=55;
			TIM8->CNT=150;
			edit_Alpha();
			set_pwm_control();
			display_pwm=1;
			asserv_pos_fixe();
			flag_bb=0;

		}
	}


	if (state_cnc==22)
	{
		if (flag_bb==1)
		{
			state_cnc=80;
			flag_bb=0;
		}
	}
	if (state_cnc==20){
		switch(display_cursor)
		{

		//Commande simple
			case 1:
				if (flag_bb==1){
					state_cnc=2;
					flag_bb=0;
				}
			break;

			//Commande décallée
			case 2:
				if (flag_bb==1){
					state_cnc=3;
					flag_bb=0;
				}
			break;

			//Speed regulation
			case 3:
				if (flag_bb==1){
					disp_reset();
					state_cnc=4;
					HAL_ADCEx_InjectedStart_IT(&hadc1);
					HAL_ADCEx_InjectedStart_IT(&hadc2);
					flag_bb=0;
				}
			break;

			//Asservissement de position
			case 4:
				if (flag_bb==1)
				{
					disp_reset();
					state_cnc=5;
					flag_bb=0;
				}
			break;
		}

	}
	if (state_cnc==55){
		if (flag_bb==1){
			disp_reset();
			set_pwm_control();
			display_pwm=0;
			TIM2->CNT=0;
			state_cnc=5;
			flag_bb=0;
		}
	}
	if (state_cnc==33){
		if (flag_bb==1){
			state_cnc=20;
			flag_bb=0;
		}
	}



}
void edit_Alpha()
{
	alpha=TIM8->CNT;
	alpha=alpha%100;
}
void fsm_cnc()
{

	switch(state_cnc)
	{
		//Init déjà fait dans le setup
		case 0:


		break;

		//Page phase test fixe
		case 1:

			if (flag_display==1)
			{
				// Page affichage de la phase de test
				affichage_page_de_test_fixe();
				//affichage_page_de_test_value();
				flag_display=0;
				state_cnc=11;
			}

		break;

		//test
		case 2:
			if (flag_display==1){
				//On fixe la valeur du alpha afin de commencer à 50% + une marge de sécurité
				TIM8->CNT=150;
				edit_Alpha();


				//Activation de la pwm
				cnc_TIM1_Init();
				set_pwm_control();
				display_pwm=1;



				//Affichage des valeurs sur le LCD
				disp_motor_drive_fixe();

				//Reset des param
				state_cnc=22;
				flag_display=0;
			}
		break;


		//Commande décalée
		case 3:
			if (flag_display==1)
			{
				//On fixe la valeur du alpha afin de commencer à 50% + une marge de sécurité
				TIM8->CNT=150;
				edit_Alpha();

				//Activation de la pwm
				//cnc_TIM1_Init_offset();
				set_pwm_control();
				display_pwm=1;

				//Affichage des valeurs sur le LCD
				disp_motor_drive_fixe_dec();

				//Reset des param
				state_cnc=33;
				flag_display=0;
			}
		break;


		//Speed Regulation
		case 4:
			if (flag_display==1){
				choix_consigne();
				//On fixe la valeur du alpha afin de commencer à 50% + une marge de sécurité



				//Activation de la pwm



				//Affichage des valeurs sur le LCD


				//Reset des param


				flag_display=0;
			}
		break;

		//Asservissement de position angulaire en degré
		case 5:
			if (flag_display==1){
				choix_consigne_pos();
				flag_display=0;
			}
		break;

		//Menu déroulant
		case 20:
			if (flag_display==1)
			{
				edit_cursor();
				affichage_menu();
				cnc_lcd_i2c_goto(display_cursor,1);
				flag_display=0;
			}
		break;


		//Page phase test dynamique
		case 11:
			if (flag_display==1)
			{
				//flag_disp_enc

					affichage_page_de_test_value();
					flag_state=0;

					flag_display=0;

			}
		break;

		case 22:
			if (flag_display==1)
			{
				disp_motor_drive_value();
				flag_display=0;
				display_vm();
			}

		break;

		case 33:
			if (flag_display==1)
			{
				disp_motor_drive_offset_value();

				// Compute the average and display on the LCD
				cnc_mcc_compute_I2_average();
			    cnc_mcc_display_output_current_I2_mA_average();
				flag_display=0;
				display_vm();
			}
		break;

		case 44:
			if (flag_display==1)
			{
				bo_asservi();
				flag_display=0;
				//display_vm();
				//display_current();
			}
		break;

		case 55:
			if (flag_display==1){
				position_asserv();
				flag_display=0;
			}
		break;
		// Arret de la pwm
		case 80:
			set_pwm_control();
			display_pwm=0;
			disp_reset();
			state_cnc=20;
			flag_display=0;
		break;

	}
	//Permet de changer l'état de la machine à état
	transit();
}


//Initialiser tout le système
void setup_syst()
{
	HAL_TIM_Base_Start_IT(&htim6); //timer pour le lcd
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);//timer pour l'encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//timer pour l'encoder

	// ADC Calibration
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	cnc_lcd_i2c_init();

	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(10," LCD OK ! ");
	cnc_lcd_i2c_goto(2,1);
	cnc_lcd_i2c_transmit_string(10," STM OK ! ");
	cnc_lcd_i2c_goto(4,1);
	cnc_lcd_i2c_transmit_string(21," Press BB for next.. ");
}
void affichage_page_de_test_value()
{

	if (flag_disp_enc==1){
		//disp_Encoder(buffStr1);
		flag_disp_enc=0;
	}
		disp_chrono(buffStr);
		disp_Encoder(buffStr1);
		//disp_Potent(buffStr2, hadc1);




}
void affichage_page_de_test_fixe()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(14," Hardware Test ");
	cnc_lcd_i2c_goto(2,1);
	cnc_lcd_i2c_transmit_string(9,"Chrono : ");
	cnc_lcd_i2c_goto(3,1);
	cnc_lcd_i2c_transmit_string(9,"encoder :");
	cnc_lcd_i2c_goto(4,1);

	cnc_lcd_i2c_transmit_string(8,"potent :");
	cnc_lcd_i2c_goto(4,9);
	cnc_lcd_i2c_transmit_string(12,"            ");
}
void disp_motor_drive_fixe()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(14," MCC BO Simple ");
	cnc_lcd_i2c_goto(2,1);


	//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,50*(htim1.Init.Period + 1)/100);

	cnc_lcd_i2c_transmit_string(5,">alfa");
	cnc_lcd_i2c_goto(2,11);

	cnc_lcd_i2c_transmit_string(10,"freq 20.0k");
	cnc_lcd_i2c_goto(3,1);
	cnc_lcd_i2c_transmit_string(20,"          ddt  500ns");
	cnc_lcd_i2c_goto(4,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
}

void disp_motor_drive_value()
{
	cnc_lcd_i2c_goto(2,6);
	edit_Alpha();
	char str[23]="%";
	TIM1->CCR1=alpha*TIM1->ARR/100;
	TIM1->CCR2=TIM1->CCR1;
	sprintf(buffStr,"%3d%s",alpha,str);
	cnc_lcd_i2c_transmit_string(4,buffStr);
}

void set_pwm_control()
{
	if (display_pwm==0)
	{
		//lancement timer relié à la PWM
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

		//Cas ou la pwm n'est pas genere
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

	}
	if (display_pwm==1)
	{
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

	}
}

void affichage_menu()
{
	if (display_menu==0){
		cnc_lcd_i2c_goto(1,1);
		cnc_lcd_i2c_transmit_string(20,"Commande Simple     ");
		cnc_lcd_i2c_goto(2,1);
		cnc_lcd_i2c_transmit_string(20,"Commande decalee    ");
		cnc_lcd_i2c_goto(3,1);
		cnc_lcd_i2c_transmit_string(20,"Speed Regulation    ");
		cnc_lcd_i2c_goto(4,1);
		cnc_lcd_i2c_transmit_string(20,"Asserv position deg ");
	}
	display_menu=1;

}

void edit_cursor()
{
	display_cursor=TIM8->CNT;
	display_cursor=display_cursor%4;
	display_cursor=display_cursor+1;
}

void disp_motor_drive_offset_value()
{
	cnc_lcd_i2c_goto(2,6);
	edit_Alpha();
	char str[23]="%";
	TIM1->CCR1=alpha*TIM1->ARR/100;
	TIM1->CCR2=TIM1->ARR-TIM1->CCR1;
	sprintf(buffStr,"%3d%s",alpha,str);
	cnc_lcd_i2c_transmit_string(4,buffStr);
	cnc_lcd_i2c_goto(4,11);
				//sprintf(buffStr2,"Vc %3d",V_consigne);
				sprintf(buffStr2,"%9d",TIM2->CNT);
				cnc_lcd_i2c_transmit_string(9,buffStr2);
}
void disp_motor_drive_fixe_dec()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(15," MCC BO Decalee ");
	cnc_lcd_i2c_goto(2,1);


	//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,50*(htim1.Init.Period + 1)/100);

	cnc_lcd_i2c_transmit_string(5,">alfa");
	cnc_lcd_i2c_goto(2,11);

	cnc_lcd_i2c_transmit_string(10,"freq 20.0k");
	cnc_lcd_i2c_goto(3,1);

	cnc_lcd_i2c_transmit_string(20,"          ddt  500ns");
	cnc_lcd_i2c_goto(4,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
}
void disp_reset()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
	cnc_lcd_i2c_goto(2,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
	cnc_lcd_i2c_goto(3,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
	cnc_lcd_i2c_goto(4,1);
	cnc_lcd_i2c_transmit_string(20,"                    ");
}

void cnc_TIM1_Init_offset(void)
{

	// Main parameters
		htim1.Instance = TIM1;
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;							// Up for simple PWM
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)	{ Error_Handler(); }		// Load Base Param
		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)		{ Error_Handler(); }		// Load PWM Param

		// CHANNEL 2 - Channel Parameters - Only the Mode to be changed
		TIM_OC_InitTypeDef sConfigOC = {0};										// STM32 Structure
		sConfigOC.OCMode = TIM_OCMODE_PWM2;										// Mode 2
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
			{ Error_Handler(); }												// Load Channel 2 params

		// Last Init ... copying MX_Init
		HAL_TIM_MspPostInit(&htim1);

}

void cnc_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
void display_vm() {
	t_previous = HAL_GetTick();
	cnt_previous = TIM2->CNT;  // Lire la valeur du compteur

    //if (htim->Instance == TIM2) {
     speed_rpm = 70.5 * (cnt_actual- cnt_previous)/ (t_actual- t_previous);// Calculer la vitesse en RPM
	//speed_rpm = 0.0136*(cnt_actual- cnt_previous)/ (t_actual- t_previous);
     t_actual = t_previous;
     cnt_actual = cnt_previous;

        // Afficher la vitesse sur le LCD
    	//if (flag_display==1){
    		cnc_lcd_i2c_goto(4,1);
    	    sprintf(buffStr1,"Vm: %3d RPM",speed_rpm);
    	    cnc_lcd_i2c_transmit_string(11,buffStr1);
    	    flag_display=0;

}

void choix_consigne()
{
	if (flag_first==0)
	{
		ref=TIM8->CNT;
	}
	flag_first=1;
	if (flag_display==1){
		V_consigne=TIM8->CNT-ref;
		cnc_lcd_i2c_goto(1,1);
		cnc_lcd_i2c_transmit_string(16," Choix consigne ");
		cnc_lcd_i2c_goto(3,1);
		sprintf(buffStr, " Consigne: %3d RPM",V_consigne);
		cnc_lcd_i2c_transmit_string(18,buffStr);
		flag_display=0;
		}


}
void speed_reg_fixe()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(16,"Speed Regulation");
	cnc_lcd_i2c_goto(2,1);
	cnc_lcd_i2c_transmit_string(9," erreur :");
	//cnc_lcd_i2c_goto(4,1);
	//cnc_lcd_i2c_transmit_string(14," Consigne Rpm:");

}
void bo_asservi()
{

		display_vm();
		int erreur_rpm=V_consigne-speed_rpm;
		 sum_asserv = erreur_rpm;
		 int32_t b=16*V_consigne*2/240-50;
			//alpha=(int) 50+0.5*erreur_rpm*16/Vm+0.5*16*sum_asserv/Vm;
			alpha= 50+b+0.02*erreur_rpm*16/24;

					//0.5*95*sum_asserv/Vm;
			TIM1->CCR1=alpha*TIM1->ARR/100;
			TIM1->CCR2=TIM1->ARR-TIM1->CCR1;

			cnc_lcd_i2c_goto(2,10);
			sprintf(buffStr, "%4d",erreur_rpm);
			cnc_lcd_i2c_transmit_string(4,buffStr);
			cnc_lcd_i2c_goto(3,16);
			sprintf(buffStr4, "%4d",alpha);
			cnc_lcd_i2c_transmit_string(4,buffStr4);
			cnc_lcd_i2c_goto(4,13);
			sprintf(buffStr2,"Vc %3d",V_consigne);

			cnc_lcd_i2c_transmit_string(6,buffStr2);
			//flag_display=0;


}

void display_current()
{
	int32_t current=(i1- 2130)*14660/1000;
	cnc_lcd_i2c_goto(3,1);
	sprintf(buffStr3,"i :%05d A",current);
	cnc_lcd_i2c_transmit_string(9,buffStr3);
}
void choix_consigne_pos()
{
	if (flag_first==0)
		{
			ref=TIM8->CNT;
		}
		flag_first=1;
		if (flag_display==1){
			position_cons=TIM8->CNT-ref;
			cnc_lcd_i2c_goto(1,1);
			cnc_lcd_i2c_transmit_string(16," Choix consigne ");
			cnc_lcd_i2c_goto(3,1);
			sprintf(buffStr, " Consigne: %3d deg",position_cons);
			cnc_lcd_i2c_transmit_string(18,buffStr);
			flag_display=0;
			}

}
void get_pos()
{
	position_mes=360*TIM2->CNT/2400;
	position_mes=position_mes%360;
}
void asserv_pos_fixe()
{
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(20,"Asserv position deg ");
	cnc_lcd_i2c_goto(2,1);
	cnc_lcd_i2c_transmit_string(9," erreur :");
}
void position_asserv()
{
	//choix_consigne_pos();
	get_pos();
	display_vm();
	int erreur_deg=position_cons-position_mes;
	int der=erreur_deg-last_error;
	int32_t b=16*position_cons*2/480;
				//alpha=(int) 50+0.5*erreur_rpm*16/Vm+0.5*16*sum_asserv/Vm;
	//alpha=50+ 0.13*erreur_deg*16/Vm+0.1*der;
	alpha=50+ 0.17*erreur_deg*16/48+0.1*der*16/48;
						//0.5*95*sum_asserv/Vm;
	TIM1->CCR1=alpha*TIM1->ARR/100;
	TIM1->CCR2=TIM1->ARR-TIM1->CCR1;

	cnc_lcd_i2c_goto(2,10);
	sprintf(buffStr, "%4d",erreur_deg);
	cnc_lcd_i2c_transmit_string(4,buffStr);
	cnc_lcd_i2c_goto(3,1);
	sprintf(buffStr5, "%4d",TIM2->CNT);
	cnc_lcd_i2c_transmit_string(4,buffStr5);
	cnc_lcd_i2c_goto(3,16);
	sprintf(buffStr4, "%4d",alpha);
	cnc_lcd_i2c_transmit_string(4,buffStr4);
	cnc_lcd_i2c_goto(4,13);
	sprintf(buffStr2,"Vc %3d",position_cons);

	cnc_lcd_i2c_transmit_string(6,buffStr2);
	last_error=erreur_deg;


}

//	Current I1 - 3 shunt config - Converted value in A
void cnc_mcc_display_current_I1_ampere(void)
{
	// Display the value of I1 in A
	int32_t current = (int32_t)sensor.ADC1_I1_result;
//	int32_t current = (int32_t)sensor.i1_ma;
	int32_t value = (current - sensor.current_intercept1) * sensor.current_slope_milli / 1000;
	sprintf(buffStr6, "%05d", (int)value);
	cnc_lcd_i2c_goto(3,13);
	cnc_lcd_i2c_transmit_string(5,buffStr6);
}


// Compute the average value of the current
void cnc_mcc_compute_I2_average(void)
{
	// Computing the average of the last "n_ma" samples of current
	sensor.i2_ma_sum = sensor.i2_ma_buff[0];
	for (int i = 1; i < sensor.n_ma; i++)
	{
		sensor.i2_ma_sum += sensor.i2_ma_buff[i];
	}
	sensor.i2_ma = sensor.i2_ma_sum / sensor.n_ma;
}

// Convert I2 from the digital value [0,4095] to a mA value
int cnc_mcc_convert_I_digital_to_mA(int value)
{
	int32_t current = ((int32_t)value - sensor.current_intercept2)
						* sensor.current_slope_milli / 1000;
	return (int)current;
}

void cnc_mcc_display_output_current_I2_mA_average(void)
{
	// And display
	sprintf(buffStr7, "%5d", cnc_mcc_convert_I_digital_to_mA(sensor.i2_ma));
	cnc_lcd_i2c_goto(4,14);
	cnc_lcd_i2c_transmit_string(5,buffStr7);
}

