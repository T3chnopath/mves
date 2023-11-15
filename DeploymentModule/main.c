#include "bsp_deployment.h"
#include "tx_api.h"
#include "mcan.h"
#include "deployment.h"
#include "imu.h"
#include "string.h"

// Main Thread
#define THREAD_MAIN_STACK_SIZE 512
static const uint16_t THREAD_MAIN_DELAY_MS = 10;
static TX_THREAD stThreadMain;
static uint8_t auThreadMainStack[THREAD_MAIN_STACK_SIZE];
void thread_main(ULONG ctx);

// **** Static Function Declarations ****
#define QUEUE_SIZE 20
typedef struct {
    sMCAN_Message array[QUEUE_SIZE];
    int front;
    int rear;
    int size;
} MCAN_MessageQueue;

static MCAN_MessageQueue mcanQueue;

// Blink Thread
#define THREAD_BLINK_STACK_SIZE 256
static TX_THREAD stThreadBlink;
static uint8_t auThreadBlinkStack[THREAD_BLINK_STACK_SIZE];
void thread_blink(ULONG ctx);

static sMCAN_Message mcanRxMessage;
static const uint16_t LED_BLINK_TIME = 1000;
static volatile bool newCommand = false;

// Initialize the queue
void MCAN_QueueInit(MCAN_MessageQueue *q) {
    q->front = 0;
    q->rear = -1;
    q->size = 0;
}

// Check if the queue is empty
bool MCAN_QueueEmpty(MCAN_MessageQueue *q) {
    return q->size == 0;
}

// Check if the queue is full
bool MCAN_QueueFull(MCAN_MessageQueue *q) {
    return q->size == QUEUE_SIZE;
}

// Enqueue an element
void MCAN_Enqueue(MCAN_MessageQueue *q, sMCAN_Message element) {
    if (MCAN_QueueFull(q)) {
        return;
    }
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->array[q->rear] = element;
    q->size++;
}

// Dequeue an element
sMCAN_Message MCAN_Dequeue(MCAN_MessageQueue *q) {
    if (MCAN_QueueEmpty(q)) {
        sMCAN_Message empty = {0}; // Initialize to your default empty value
        return empty;
    }
    sMCAN_Message element = q->array[q->front];
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->size--;
    return element;
}

// **** Main Code ****
int main(void)
{
    tx_kernel_enter();
}

void tx_application_define(void *first_unused_memory)
{
    tx_thread_create( &stThreadMain, 
        "thread_main", 
        thread_main, 
        0, 
        auThreadMainStack, 
        THREAD_MAIN_STACK_SIZE, 
        0,
        0, 
        0,  
        TX_AUTO_START);

    tx_thread_create( &stThreadBlink, 
        "thread_blink", 
        thread_blink, 
        0, 
        auThreadBlinkStack, 
        THREAD_BLINK_STACK_SIZE, 
        10,
        10, 
        0,
        TX_AUTO_START);
}

void thread_main(ULONG ctx)
{
    sMCAN_Message currMcanRxMessage;

    // Initialize BSP and App layer
    BSP_Init();

    MCAN_QueueInit(&mcanQueue);
    
    MCAN_Init( FDCAN1, DEV_DEPLOYMENT, &mcanRxMessage );

    DeploymentInit();
    
    while(true)
    {
        // If message in queue and DeployComm is ready to process
        if(!MCAN_QueueEmpty(&mcanQueue))
        {
            currMcanRxMessage = MCAN_Dequeue(&mcanQueue);

            // If sensor data, update IMU
            if ( currMcanRxMessage.mcanID.MCAN_CAT == SENSOR_DATA )
            {
                IMU_Update( currMcanRxMessage.mcanData );
            }

            // If first byte is 1, second byte is a deployment command. Check if Deploy isn't busy.
            else if ( currMcanRxMessage.mcanData[0] == 1 && !DeployCommBusy())
            {
                DeployCommExe((DEPLOY_COMM) currMcanRxMessage.mcanData[1]);
            }
        
            // If cannot process, requeue command
            else
            {
                MCAN_Enqueue(&mcanQueue, currMcanRxMessage);
            }
        }
        tx_thread_sleep(THREAD_MAIN_DELAY_MS);
    }
}

void thread_blink(ULONG ctx)
{
    while(true)
    {
        tx_thread_sleep(LED_BLINK_TIME);
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    }
}

void MCAN_Rx_Handler( void )
{
    // Add latest message to queue
    MCAN_Enqueue(&mcanQueue, mcanRxMessage);
}