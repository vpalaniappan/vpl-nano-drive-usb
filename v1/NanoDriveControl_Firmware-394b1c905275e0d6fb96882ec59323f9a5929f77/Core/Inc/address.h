/*
 * address.h
 *
 *  Created on: Jun 8, 2023
 */

#ifndef INC_ADDRESS_H_
#define INC_ADDRESS_H_


uint8_t getAxis( void );
uint8_t getGate( void );
void setAxis( uint8_t axis );
void setGate( uint8_t gate );

char WaitForTurn();
void GetAddress();
void SetupAddressingRoutine();
void SetupAnnounce();

#endif /* INC_ADDRESS_H_ */
