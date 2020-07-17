#ifndef HXM8_MCP2515_COMMUNICATION
#define HXM8_MCP2515_COMMUNICATION

/*BEGIN USER CONFIGURATION PARAMETERS*/
#define CAN_BUS_TASK_PRIORITY                   15  //PRIORITY OF THE CAN BUS TASK  
/*END USER CONFIGURATION PARAMETERS*/


/*BEGIN USER DEFINED MESSAGES OVER CAN*/

//forward definition
#define FOR                                     0x00664F52
#define WARD                                    0x57415244

//back definition
#define BACK                                    0x6261636B

//right direction definition
#define R                                       0x00000072
#define IGHT                                    0x69676874

//left direction definition
#define LEFT                                    0x6C656674

/*END USER DEFINED MESSAGES OVER CAN*/


/*BEGIN IDENTIFIER DEFINITIONS*/
#define ID_INFORMATION                          8
#define ID_WARNING                              9
#define ID_ERROR                                10
#define ID_CRITICAL                             11
/*END IDENTIFIER DEFINITIONS*/



/*USER DEFINED PREPROCESSOR*/
#define GPIO_INPUT_IO_0                         4
#define GPIO_INPUT_IO_1                         5
#define GPIO_INPUT_PIN_SEL                      ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
/*END USER DEFINED PREPROCESSOR*/


void hxm8_mcp2515_init(void);

#endif