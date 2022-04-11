#include <stdio.h>
#include <unistd.h>
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/point32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "custom.h"

#include <coremark.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// /*< Stores the period of the task in ticks */
// extern TickType_t xTaskPeriods[10];
// /*< Stores the relative deadline of the task for implicit deadline EDF scheduling in ticks */
// extern TickType_t xTaskDeadlines[10];

rcl_publisher_t publisher;

TaskHandle_t Task01Handle;
TaskHandle_t Task02Handle;
TaskHandle_t Task03Handle;
TaskHandle_t Task04Handle;

TickType_t xTaskPeriods[5];
TickType_t xTaskDeadlines[5];
uint8_t xTaskType[5];
const BaseType_t numberOfTasks = sizeof( xTaskPeriods ) / sizeof( xTaskPeriods[0] );
BaseType_t scalingFactor = 0.65;

customParameters_t Task00_parameters = {
  .budget = 0,
  .period = 100,
  .deadline = 100,
};
customParameters_t Task01_parameters = {
  .budget = 1000/4,
  .period = 3000,
  .deadline = 3000,
};
customParameters_t Task02_parameters = {
  .budget = 2000/4,
  .period = 9000,
  .deadline = 9000,
};
customParameters_t Task03_parameters = {
  .budget = 5000/4,
  .period = 25000,
  .deadline = 25000,
};
customParameters_t Task04_parameters = {
  .budget = 1500/4,
  .period = 15000,
  .deadline = 15000,
};

void handleFault(void);
void coremark_run(uint32_t iterations);
void busyDelaySeconds(uint32_t seconds);

void thread0(void * argument)
{
	// Fault Simulator
	static uint8_t status = true;
	customParameters_t *xtaskParameters;
	xtaskParameters = ( customParameters_t * ) argument;
	const TickType_t xtaskPeriod = xtaskParameters->period;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){
		if (status)
		{
			if (xLastWakeTime >= 21000)
			{
				handleFault();
				status = false;
			}
		}
		vTaskDelayUntil(&xLastWakeTime, xtaskPeriod);
	}
}

void thread1(void * argument)
{
	customParameters_t *xtaskParameters;
	xtaskParameters = ( customParameters_t * ) argument;
	const TickType_t xtaskPeriod = xtaskParameters->period;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	geometry_msgs__msg__Point32 msg;
    geometry_msgs__msg__Point32__init(&msg);
	msg.x = 0;
	msg.y = 0;
	msg.z = (float) 1;
	while(1){
		msg.x = (float) xTaskGetTickCount();
		coremark_run(xtaskParameters->budget);
		msg.y = (float) xTaskGetTickCount();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		vTaskDelayUntil(&xLastWakeTime, xtaskPeriod);
	}
}

void thread2(void * argument)
{
	customParameters_t *xtaskParameters;
	xtaskParameters = ( customParameters_t * ) argument;
	const TickType_t xtaskPeriod = xtaskParameters->period;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	geometry_msgs__msg__Point32 msg;
    geometry_msgs__msg__Point32__init(&msg);
	msg.x = 0;
	msg.y = 0;
	msg.z = (float) 2;
	while(1){
		msg.x = (float) xTaskGetTickCount();
		coremark_run(xtaskParameters->budget);
		msg.y = (float) xTaskGetTickCount();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		vTaskDelayUntil(&xLastWakeTime, xtaskPeriod);
	}
}

void thread3(void * argument)
{
	customParameters_t *xtaskParameters;
	xtaskParameters = ( customParameters_t * ) argument;
	const TickType_t xtaskPeriod = xtaskParameters->period;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	geometry_msgs__msg__Point32 msg;
    geometry_msgs__msg__Point32__init(&msg);
	msg.x = 0;
	msg.y = 0;
	msg.z = (float) 3;
	while(1){
		msg.x = (float) xTaskGetTickCount();
		coremark_run(xtaskParameters->budget);
		msg.y = (float) xTaskGetTickCount();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		vTaskDelayUntil(&xLastWakeTime, xtaskPeriod);
	}
}

void thread4(void * argument)
{
	customParameters_t *xtaskParameters;
	xtaskParameters = ( customParameters_t * ) argument;
	const TickType_t xtaskPeriod = xtaskParameters->period;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	geometry_msgs__msg__Point32 msg;
    geometry_msgs__msg__Point32__init(&msg);
	msg.x = 0;
	msg.y = 0;
	msg.z = (float) 4;
	while(1){
		msg.x = (float) xTaskGetTickCount();
		coremark_run(xtaskParameters->budget);
		msg.y = (float) xTaskGetTickCount();
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		vTaskDelayUntil(&xLastWakeTime, xtaskPeriod);
	}
}


void appMain(void * argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "freertos_ros2_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
		"topic"));
	
	xTaskPeriods[0] = Task00_parameters.period;
	xTaskPeriods[1] = Task01_parameters.period;
	xTaskPeriods[2] = Task02_parameters.period;
	xTaskPeriods[3] = Task03_parameters.period;
	xTaskPeriods[4] = Task04_parameters.period;
	xTaskDeadlines[0] = Task00_parameters.deadline;
	xTaskDeadlines[1] = Task01_parameters.deadline;
	xTaskDeadlines[2] = Task02_parameters.deadline;
	xTaskDeadlines[3] = Task03_parameters.deadline;
	xTaskDeadlines[4] = Task04_parameters.deadline;
	xTaskType[0] = 'L';
	xTaskType[1] = 'L';
	xTaskType[2] = 'H';
	xTaskType[3] = 'H';
	xTaskType[4] = 'L';
	
	xTaskCreate(thread0, "rtos_thread_0",  500, &Task00_parameters, (osPriority_t) osPriorityRealtime1, NULL);
	xTaskCreate(thread1, "rtos_thread_1", 1500, &Task01_parameters, (osPriority_t) osPriorityRealtime, &Task01Handle);
	xTaskCreate(thread2, "rtos_thread_2", 1500, &Task02_parameters, (osPriority_t) osPriorityRealtime, &Task02Handle);
	xTaskCreate(thread3, "rtos_thread_3", 1500, &Task03_parameters, (osPriority_t) osPriorityRealtime, &Task03Handle);

	while(1){
		sleep(100);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))

	RCCHECK(rcl_node_fini(&node))

  	vTaskDelete(NULL);
}

void handleFault()
{
	// vTaskDelete(Task01Handle);
	vTaskDelete(Task02Handle);
	vTaskDelete(Task03Handle);
	scalingFactor = 1;
	xTaskCreate(thread4, "rtos_thread_4", 1500, &Task04_parameters, (osPriority_t) osPriorityRealtime, &Task04Handle);
	xTaskCreate(thread3, "rtos_thread_3", 1500, &Task03_parameters, (osPriority_t) osPriorityRealtime, &Task03Handle);
	xTaskCreate(thread2, "rtos_thread_2", 1500, &Task02_parameters, (osPriority_t) osPriorityRealtime, &Task02Handle);
}


void coremark_run(uint32_t iterations)
{
  core_results result;
  volatile ee_s32 seed1_volatile=0x0;
  volatile ee_s32 seed2_volatile=0x0;
  volatile ee_s32 seed3_volatile=0x66;
  ee_u32 i;
  ee_u16 crc;
  ee_u8 stack_memblock[TOTAL_DATA_SIZE];

  result.seed1 = seed1_volatile;
  result.seed2 = seed2_volatile;
  result.seed3 = seed3_volatile;
  result.iterations = iterations;
  result.execs = (1<<0)|(1<<1)|(1<<2);
  result.memblock[0]=stack_memblock;
  result.size=TOTAL_DATA_SIZE/NUM_ALGORITHMS;
  result.err=0;

  result.memblock[1]=(char *)(result.memblock[0])+result.size*0;
  result.memblock[2]=(char *)(result.memblock[0])+result.size*1;
  result.memblock[3]=(char *)(result.memblock[0])+result.size*2;
  result.list=core_list_init(result.size,result.memblock[1],result.seed1);
  core_init_matrix(result.size, result.memblock[2], (ee_s32)result.seed1 | (((ee_s32)result.seed2) << 16), &(result.mat) );
  core_init_state(result.size,result.seed1,result.memblock[3]);

  result.crclist=0;
  result.crcmatrix=0;
  result.crcstate=0;

  for (i=0; i<result.iterations; i++) {
    crc=core_bench_list(&result,1);
    result.crc=crcu16(crc,result.crc);
    crc=core_bench_list(&result,-1);
    result.crc=crcu16(crc,result.crc);
    if (i==0) result.crclist=result.crc;
  }

}

void busyDelaySeconds(uint32_t seconds)
{
	volatile uint32_t i, j;
	for(j=0; j<seconds; j++){
		for(i=0; i<=15*1000*1000; i++);
	}
}

