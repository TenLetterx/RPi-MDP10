#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#include <getConversion.h>
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "commands_FLAGS.h"

enum _cmdOpType {
	DRIVE,			//is a drive command, i.e., robot will move.

	//Commands related to relaying information.
	INFO_DIST,		//toggle start/stop of accumulative distance tracking.
	INFO_MARKER,		//arbitary marker.
	TURN_IN_PLACE  // New drive mode
};
enum _cmdDistType {
	TARGET,			//drive for this distance
	STOP_AWAY,		//stop when roughly this distance away from front.
	STOP_L,			//stop when left IR sensor is more than threshold.
	STOP_R,			//stop when right IR sensor is more than threshold.
	STOP_L_LESS,
	STOP_R_LESS
};

typedef enum _cmdOpType CmdOpType;
typedef enum _cmdDistType CmdDistType;

struct command_t {
	//command op type
	CmdOpType opType;

	//command string
	uint8_t shouldSend; 	//if command should be tracked for finishing.
	uint8_t *str;
	uint8_t str_size;

	/* start: DRIVE parameters */
	//motor directives
	int8_t dir;				//-1: backward, 0: stop, 1: forward
	uint8_t speed;			//0 to 100
	float angleToSteer;	//-25 to 25

	//distance directives
	CmdDistType distType;
	float val;				//for angle != 0: angle to turn; for angle = 0: distance to drive.
	/* end: DRIVE parameters */

	struct command_t *next;
};

typedef struct command_t Command;

void commands_process(UART_HandleTypeDef *uart, uint8_t *buf, uint8_t size);
Command *commands_pop();
Command *commands_peek();
Command *commands_peek_next_drive();
void commands_end(UART_HandleTypeDef *uart, Command *cmd);
uint8_t commands_type_match(Command *a, Command *b);
#endif /* INC_COMMANDS_H_ */
