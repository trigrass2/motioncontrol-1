#ifndef OBJECTDICTIONARY_H
#define OBJECTDICTIONARY_H


#define controlWord            0x6040,0x00
#define statusWord             0x6041,0x00

#define operationMode          0x6060,0x00
#define operationModeDisplay   0x6061,0x00

#define  targetPosition        0x607A,0x00
#define  positionActualVal     0x6064,0x00
#define  positionDemand        0x6062,0x00
#define  positonFollowingError 0x60F4,0X00
#define  positonCounts		   0x6063, 0x00

#define  velocityActvalue       0x6069,0x00
#define  velocityAddress        0x6069,0x00
#define  targetVelocity         0x60FF,0x00
#define	 velocityOffset			0x60b1, 0x00

#define profileVelocity                    0x6081,0x00
#define profileAcceleration                0x6083,0x00
#define profileDeceleration                0x6084,0x00
#define quickStopDeceleration              0x6085,0x00
#define motionProfileType                  0x6086,0x00
#define linearRampTrapezoidal              0x00,0x00
#define velocityEncoderResolutionNum       0x6094,0x01
#define velocityEncoderResolutionDen       0x6094,0x02

#define dcCircuitLinkVoltage	   0x6079,0x00
#define targetTorque               0x6071,0x00 
#define maxTorque                  0x6072,0x00
#define torqueActualValue		   0x6077,0x00

#define maxCurrent				   0x6073, 0x00
#define currentActualValue		   0x6078, 0x00
#define errorCode				   0x603f, 0x00   // 2 bit

#define extraStatusRegister		   0x2085, 0x00

// State machine parameters; 

#define commReset              0x81
#define fullReset              0x82
#define start                  0x01
#define goreadyToSwitchOn      0x06
#define goSwitchOn             0x07
#define goEnable               0X0F
#define goSwitchonDisable      0x00
#define run                    0x1F
#define expedite               0x3F       //like run, but dont finish actual position profile
#define quickStop              0x02 


/* From CiA402, page 27
	Table 30 - State coding
	Statusword      |      PDS FSA state
xxxx xxxx x0xx 0000 | Not ready to switch on
xxxx xxxx x1xx 0000 | Switch on disabled
xxxx xxxx x01x 0001 | Ready to switch on
xxxx xxxx x01x 0011 | Switched on
xxxx xxxx x01x 0111 | Operation enabled
xxxx xxxx x00x 0111 | Quick stop active
xxxx xxxx x0xx 1111 | Fault reaction active
xxxx xxxx x0xx 1000 | Fault
*/
#define FSAFromStatusWord(SW) (SW & 0x006f)
#define NotReadyToSwitchOn   0b00000000
#define NotReadyToSwitchOn2  0b00100000
#define SwitchOnDisabled     0b01000000
#define SwitchOnDisabled2    0b01100000
#define ReadyToSwitchOn      0b00100001
#define SwitchedOn           0b00100011
#define OperationEnabled     0b00100111
#define QuickStopActive      0b00000111
#define FaultReactionActive  0b00001111
#define FaultReactionActive2 0b00101111
#define Fault                0b00001000
#define Fault2               0b00101000

// SatusWord bits :
#define SW_ReadyToSwitchOn     0x0001
#define SW_SwitchedOn          0x0002
#define SW_OperationEnabled    0x0004
#define SW_Fault               0x0008
#define SW_VoltageEnabled      0x0010
#define SW_QuickStop           0x0020
#define SW_SwitchOnDisabled    0x0040
#define SW_Warning             0x0080
#define SW_Remote              0x0200
#define SW_TargetReached       0x0400
#define SW_InternalLimitActive 0x0800

// ControlWord bits :
#define SwitchOn        0x0001
#define EnableVoltage   0x0002
#define QuickStop       0x0004
#define EnableOperation 0x0008
#define FaultReset      0x0080
#define Halt            0x0100
/* CiA402 statemachine definition end */

#define positionMode            0x01
#define velocityMode            0x03
#define torqueMode              0x04

#define checkError              0x1002,0x00
#define quickStopMode           0x605A,0x00
#define stopOptionCode          0x605D,0x00

#endif 
