/*
 * position.h
 *
 *  Created on: May 31, 2023
 */

#ifndef INC_POSITION_H_
#define INC_POSITION_H_

#define MAX_POSITION_POS 200000000000LL
#define MAX_POSITION_NEG 900000000000LL
#define SYSTEM_MAX_MOVE 200000000000000LL	//Right now, our system max move is set at 200 meters. And our travel length is set at +/- 100m.
#define SYS_TLP 100000000000000LL
#define SYS_TLN -100000000000000LL

extern int64_t CurrentPos;

void PositionHandler(void);
char CheckMove(int64_t, enum CommandList);
char CheckPos(int64_t, enum CommandList);
char CheckLimitConfig(uint8_t);
char CheckNewLimit(int64_t, int64_t);
void PositionUpdate();

enum MoveTypes getMoveType(uint8_t);
enum MoveTypes getJogType();

#endif /* INC_POSITION_H_ */
