#ifndef BLESERIAL_H
#define BLESERIAL_H

#include "BirdBrain.h"

/************************************************************************/
/******************     DEFINES        **********************************/
/************************************************************************/
#define SYMBOL 									 					0x80
#define SCROLL                    									0x40
#define SAMD_MINIMUM_FIRMWARE_VERSION 						0x01   

//Commands opcode
#define SETALL_SPI                                0xCA
#define SET_LEDARRAY                              0xCC
#define SET_LED_2                                 0xC1
#define SET_LED_3                                 0xC2
#define SET_BUZZER                                0xCD
#define SET_CALIBRATE                             0xCE
#define SET_FIRMWARE                              0xCF
#define STOP_ALL                                  0xCB
#define NOTIFICATIONS                             0x62
#define START_NOTIFY                              0x67
#define START_NOTIFYV2                            0x70
#define STOP_NOTIFY                               0x73

#define FINCH_SETALL_LED			              0xD0
#define FINCH_SETALL_MOTORS_MLED				  0xD2
#define FINCH_SET_FIRMWARE                        0xD4
#define FINCH_RESET_ENCODERS                      0xD5
#define FINCH_POWEROFF_SAMD                       0xD6
#define FINCH_STOPALL                             0xDF

#define BROADCAST                                 'b'
#define MICRO_IO                                  0x90

#define LENGTH_SETALL_SPI                         13
#define LENGTH_OTHER_SPI                          4
#define LENGTH_ALL_SPI                            15

#define HARDWARE_VERSION				          0x01

#define SENSOR_SEND_LENGTH                	      14
#define V2_SENSOR_SEND_LENGTH             	      16
#define FINCH_SENSOR_SEND_LENGTH                  20
#define BLE__MAX_PACKET_LENGTH                    20 

#define MIC_SAMPLES                               8

#define FULL_BATT                                 100                          //All four tail LEDS are green above this
#define BATT_THRESH1                              55							//Three tail LEDS are green above this
#define BATT_THRESH2                              40							//Two Tail LEDS are yellow above this
														//One Tail LED is red below BATT_THRESH2

void bleSerialInit(ManagedString devName);  // Initializes the UART
void bleSerialCommand(); // Checks what command (setAll, get firmware, etc) is coming over BLE, then acts as necessary
void assembleSensorData(); // Collects the notification data and sends it to the computer/tablet

void returnFirmwareData();
void playConnectSound();
void playDisconnectSound();
void flashInitials();

#endif