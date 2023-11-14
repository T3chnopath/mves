#include "maincompute.h"
#include "bsp_maincompute.h"

#include <stdbool.h>
#include "bno055.h"
#include "dc_motor.h"
#include "tx_api.h"

typedef enum{
    CCW,
    CW
} MOTOR_DIRECTION;

static MOTOR_DIRECTION Get_Direction(float y_avg){
    return (y_avg > 0) ? CCW : CW;
}

void Bay_Orientate(void){

}
