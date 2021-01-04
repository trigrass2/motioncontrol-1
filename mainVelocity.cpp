/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"
#include "ObjectDictionary.h"
/****************************************************************************/

// Application parameters
#define FREQUENCY  1000                         //Set the frequency in here.
#define CLOCK_TO_USE CLOCK_MONOTONIC      // A nonsettable system-wide clock that represents monotonic time/ \
                                            since—as described by POSIX—"some unspecified point in the \
                                            past".  On Linux, that point corresponds to the number of seconds\
                                            that the system has been running since it was booted.
//#define MEASURE_TIMING
#define MAX_SAFE_STACK (4096 * 4096)
/****************************************************************************/
#define BIT_TEST(NUM,N) (NUM && (1<<N))

#define NSEC_PER_SEC (1000000000L)                              // Period mesurements in nanoseconds.
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)                    // Period defined in here based on frequency 1sc/freq

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)
       //Since timespec structure consists of sec and ns addition and difference requires new function
        

        // Timespec structure to nanoseconds function.
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)       
#define TEST_BIT(num,N)     (num && (1<<N))
#define BIT_SET(num,N)      (num | (1<<N))
#define BIT_RESET(num,N)    (num & ~(1<<N))
/****************************************************************************/

// EtherCAT 
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *masterDomain = NULL;
static ec_domain_state_t masterDomain_state = {};

static ec_slave_config_t *sc = NULL;
static ec_slave_config_state_t sc_state = {};
/****************************************************************************/
#define HOME_POS 0
#define COUNTS_PER_REVOLUTION 512*19 // 11 is gear reduction for the motor.
#define PROFILE_VELOCITY 3
// process data
static unsigned int counter = FREQUENCY;
// process data
static uint8_t *SERVO_DRIVE_DOMAIN = NULL;             //domaains for process data excange for individual slaves

#define DRIVE_POS  0, 0                    // $ ethercat slaves returns slave position based on their connection queue

#define GOLD_SOLO_DRIVE 0x0000009a, 0x00030924  // ethercat cstruct will retrun the necessary information for slave adresses and values.          
                                                // slave vendor id and product code.

// offsets for PDO entries
static struct {
    // INPUTs ; from slave to the master ; 
 uint32_t POSITION_ACTUAL_VALUE;
 uint32_t POSITION_FOLLOWING_ERROR;
 uint32_t TORQUE_ACTUAL_VALUE;
 uint32_t STATUS_WORD;
 uint32_t  CURRENT_OP_MODE;
 uint32_t POSITION_COUNTS;            // Counts;
 uint32_t VELOCITY_COUNTS_PER_SEC;     // [Counts/sec]
 uint32_t CURRENT_ACTUAL_VALUE;
 uint32_t ERROR_CODE;
 //-----------------------------------------------
 // OUTPUTS ; from master to the slave ; 
 uint32_t TARGET_POSITION;
 uint32_t TARGET_VELOCITY;           
 uint32_t TARGET_TORQUE;
 uint32_t MAX_TORQUE;
 uint32_t CONTROL_WORD;
 uint32_t  OPERATION_MODE;
 uint32_t MAX_CURRENT;
 uint32_t PROFILE_ACCELERATION;
 uint32_t PROFILE_DECELERATION;
 uint32_t VELOCITY_OFFSET;
}offset;

static unsigned int     sync_ref_counter = 0;

const struct timespec cycletime = {0, PERIOD_NS};       // cycletime settings in ns. 

const static ec_pdo_entry_reg_t masterDomain_regs[] = {
    // INPUT PDO MAPPING ;
    {DRIVE_POS,  GOLD_SOLO_DRIVE, positionActualVal, &offset.POSITION_ACTUAL_VALUE},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x60F4, 0X00, &offset.POSITION_FOLLOWING_ERROR},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6077, 0X00, &offset.TORQUE_ACTUAL_VALUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, statusWord, &offset.STATUS_WORD},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6061, 0x00, &offset.CURRENT_OP_MODE},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6063, 0x00, &offset.POSITION_COUNTS},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6069, 0X00, &offset.VELOCITY_COUNTS_PER_SEC},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6078, 0x00, &offset.CURRENT_ACTUAL_VALUE},
    //{DRIVE_POS,  GOLD_SOLO_DRIVE, 0x603f, 0x00, &offset.ERROR_CODE},
    //-----------------------------------------------------------------
    // OUTPUT PDO Mapping ; 
    {DRIVE_POS,  GOLD_SOLO_DRIVE, targetPosition, &offset.TARGET_POSITION},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x60ff, 0x00, &offset.TARGET_VELOCITY},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6071, 0x00, &offset.TARGET_TORQUE},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6072, 0x00, &offset.MAX_TORQUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, controlWord, &offset.CONTROL_WORD},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6060, 0x00, &offset.OPERATION_MODE},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6073, 0x00, &offset.MAX_CURRENT},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6083, 0x00, &offset.PROFILE_ACCELERATION},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x6084, 0x00, &offset.PROFILE_DECELERATION},
   // {DRIVE_POS,  GOLD_SOLO_DRIVE, 0x60b1, 0x00, &offset.VELOCITY_OFFSET}, 
    {}
};

/******************************* $ ethercat cstruct returns this info in below ************************************/
/* Master 0, Slave 0
 * Vendor ID:       0x0000009a
 * Product code:    0x00030924
 * Revision number: 0x00010420
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
     // Outputs from master to slave , WRITE to this indexes
    {0x607a, 0x00, 32},    // TARGET_POSITION
    {0x60ff, 0x00, 32},    // TARGET_VELOCITY
    {0x6071, 0x00, 16},    // TARGET_TORQUE
    {0x6072, 0x00, 16},    // MAX_TORQUE
    {0x6040, 0x00, 16},    // CONTROL_WORD
    {0x6060, 0x00, 8},     // MODE_OF_OPERATION
    {0x0000, 0x00, 8}, /* Gap */
    {0x6073, 0x00, 16},     // MAX_CURRENT 
    {0x6083, 0x00, 32},     // PROFILE_ACCELERATION
    {0x6084, 0x00, 32},     // PROFILE_DECELERATION
    {0x60b1, 0x00, 32},     // VELOCITY_OFFSET

    // Inputs from slave to the master READ from this indexes
    {0x6064, 0x00, 32},     // POSITION_ACTUAL_VALUE
    {0x60f4, 0x00, 32},     // POSITION_FOLLOWING_ERROR_ACTUAL_VALUE
    {0x6077, 0x00, 16},     // TORQUE_ACTUAL_VALUE
    {0x6041, 0x00, 16},     // STATUS_WORD
    {0x6061, 0x00, 8},      // CUR_MODE_OF_OPERATION
    {0x0000, 0x00, 8}, /* Gap */
    {0x6063, 0x00, 32},     // actual_position (COUNTS)
    {0x6069, 0x00, 32},     // VELOCITY_SENSOR_ACTUAL_VALUE (COUTNS/SEC)
    {0x6077, 0x00, 16},     // TORQUE_ACTUAL_VALUE
    {0x6078, 0x00, 16},     // CURRENT_ACTUAL_VALUE
    {0x603f, 0x00, 16},     // ERROR_CODE
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 3, slave_0_pdo_entries + 0},
    {0x1a00, 3, slave_0_pdo_entries + 3},
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};




/***************************  EOF ethercat cstruct *******************************************/

 /*timespec consists of seconds and nanoseconds, additin 
    requires external function. We use this functions 
    for time specific measurements 
*/
struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_masterDomain_state(void)
{
    ec_domain_state_t ds;            //Domain instance

    ecrt_domain_state(masterDomain, &ds);

    if (ds.working_counter != masterDomain_state.working_counter)
        printf("masterDomain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != masterDomain_state.wc_state)
        printf("masterDomain: State %u.\n", ds.wc_state);

    masterDomain_state = ds;
}

/*****************************************************************************/

int check_master_state()
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
        if (ms.slaves_responding < 1) 
        {
        printf("Connection error, only %d slaves responding",ms.slaves_responding);
        return 0;
        }
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        if(!ms.link_up) 
        return 0;
    }
    master_state = ms;
    return 1;
}


int check_slave_config_states()
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc, &s);

    if (s.al_state != sc_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }
    
     sc_state = s;
     return 1;
}
/****************************************************************************/

void *cyclic_task(void *arg)
{

struct timespec wakeupTime, time;
        #ifdef MEASURE_TIMING
            struct timespec startTime={}, endTime={}, lastStartTime = {};
            uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
            latency_min_ns = 0, latency_max_ns = 0,
            period_min_ns = 0, period_max_ns = 0,
            exec_min_ns = 0, exec_max_ns = 0,
            max_period=0, max_latency=0,
            max_exec=0, min_latency=0xffffffff,
            min_period=0xffffffff, min_exec=0xffffffff;                    
        #endif

    // get current time
clock_gettime(CLOCK_TO_USE, &wakeupTime);
uint32_t command=0x004F;
int begin=10;
unsigned int actual_position=0;
uint16_t status;
int32_t T_position = 1e6;
int8_t cur_op_mode;
int8_t new_op_mode;
uint32_t c_word;
    while(1) 
 {
            
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

        #ifdef MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &startTime);
            latency_ns = DIFF_NS(wakeupTime, startTime);
            period_ns = DIFF_NS(lastStartTime, startTime);
            exec_ns = DIFF_NS(lastStartTime, endTime);
            lastStartTime = startTime;  
            if(!begin)
            {
            if(latency_ns > max_latency)        max_latency = latency_ns;
            if(period_ns > max_period)          max_period  = period_ns;
            if(exec_ns > max_exec)              max_exec    = exec_ns;
            if(latency_ns < min_latency)        min_latency = latency_ns;
            if(period_ns < min_period )         min_period  = period_ns;
            if(exec_ns < min_exec)              min_exec    = exec_ns;
            }

            if (latency_ns > latency_max_ns)  {
                latency_max_ns = latency_ns;
            }
            if (latency_ns < latency_min_ns) {
                latency_min_ns = latency_ns;
            }
            if (period_ns > period_max_ns) {
                period_max_ns = period_ns;
            }
            if (period_ns < period_min_ns) {
                period_min_ns = period_ns;
            }
            if (exec_ns > exec_max_ns) {
                exec_max_ns = exec_ns;
            }
            if (exec_ns < exec_min_ns) {
                exec_min_ns = exec_ns;
            }
        #endif

            // receive process data
    actual_position = EC_READ_U32(SERVO_DRIVE_DOMAIN+offset.POSITION_ACTUAL_VALUE);
    status = EC_READ_U16(SERVO_DRIVE_DOMAIN+offset.STATUS_WORD);
    //cur_op_mode = EC_READ_U8(SERVO_DRIVE_DOMAIN+offset.CURRENT_OP_MODE);
    ecrt_master_receive(master);
    ecrt_domain_process(masterDomain);

    // check process data state (optional)
    check_masterDomain_state();

    if (counter) 
    {
        counter--;
    } 
    else
    { 
        // do this at 1 Hz
        counter = FREQUENCY;
        // check for master state (optional)
        if(!check_master_state()) 
      {
        printf("Error occured slave state error ; lost connection, master state error\n");
         return NULL;
      }

    if (!check_slave_config_states())
      {
        printf("Error occured slave state error ; lost connection, slave config error \n");
        return NULL;
    }
  
        #ifdef MEASURE_TIMING
                // output timing stats
                printf("-----------------------------------------------\n");
                printf("Tperiod   min   : %10u ns  | max : %10u ns\n",
                        period_min_ns, period_max_ns);
                printf("Texec     min   : %10u ns  | max : %10u ns\n",
                        exec_min_ns, exec_max_ns);
                printf("Tlatency  min   : %10u ns  | max : %10u ns\n",
                        latency_min_ns, latency_max_ns);
                printf("Tjitter max     : %10u ns  \n",
                        latency_max_ns-latency_min_ns);

                printf("Tperiod min     : %10u ns  | max : %10u ns\n",
                        period_min_ns, max_period);
                 printf("Texec  min      : %10u ns  | max : %10u ns\n",
                        exec_min_ns, max_exec);               
                 printf("Tjitter min     : %10u ns  | max : %10u ns\n",
                          latency_max_ns-latency_min_ns, max_latency-min_latency);
                
                printf("-----------------------------------------------\n");
                period_max_ns = 0;
                period_min_ns = 0xffffffff;
                exec_max_ns = 0;
                exec_min_ns = 0xffffffff;
                latency_max_ns = 0;
                latency_min_ns = 0xffffffff;
        #endif

                // calculate new process data
                printf("Status value    : 0x%x \n", status);
                printf("Position value  :  %d \n",actual_position);
                printf("Target position :  %d \n",T_position);
                printf("Error in position : %d \n",T_position-actual_position);
                printf("\n\n");
                if( (actual_position >= T_position-100) && (actual_position <= T_position+100)){
                T_position +=1e4;
                printf("Target reached new target : %d \n",T_position);
                }
                
                //DS402 CANOpen over EtherCAT state machine
            if( (status & command) == 0x0040  )  // If status is "Switch on disabled", \
                                                   change state to "Ready to switch on"
            {
                c_word  = goreadyToSwitchOn;
                command = 0x006f;
                printf("Transiting to -Ready to switch on state...- \n");
                EC_WRITE_U16(SERVO_DRIVE_DOMAIN + offset.CONTROL_WORD, c_word);
            }

            else if( (status & command) == 0x0021) // If status is "Ready to switch on", \
                                                         change state to "Switched on"
            {
                c_word  = goSwitchOn;     
                command = 0x006f;
                printf("Transiting to -Switched on state...- \n");            
                EC_WRITE_U16(SERVO_DRIVE_DOMAIN + offset.CONTROL_WORD, c_word);                                                   
            }

            else if( (status & command) == 0x0023)         // If status is "Switched on", \
                                                         change state to "Enable "                                     
            {
                printf("Operation enabled...\n");
                c_word  = goEnable;
                command = 0x006f;
                EC_WRITE_U16(SERVO_DRIVE_DOMAIN + offset.CONTROL_WORD, c_word);
                EC_WRITE_S32(SERVO_DRIVE_DOMAIN + offset.TARGET_POSITION, T_position);
            }else if( (status & command) == 0x0027){        // Operation position mode ; set new-position;
                c_word=run;
                EC_WRITE_U16(SERVO_DRIVE_DOMAIN + offset.CONTROL_WORD, c_word);
            }
            else if ((status & 0x4f) == 0X08)
            {
                command = 0X04f;
                printf("ERROR : Motor fault state ... \n"
                           "Resetting Motor state\n");
                c_word = 0XFFFF;
                EC_WRITE_U16(SERVO_DRIVE_DOMAIN + offset.CONTROL_WORD, c_word);
            }
    }

            // write process data
            EC_WRITE_S32(SERVO_DRIVE_DOMAIN + offset.TARGET_POSITION, T_position);
            if (sync_ref_counter) {
                sync_ref_counter--;
            } else {
                sync_ref_counter = 1; // sync every cycle

                clock_gettime(CLOCK_TO_USE, &time);
                ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
            }
            ecrt_master_sync_slave_clocks(master);
            // send process data
            ecrt_domain_queue(masterDomain);
            ecrt_master_send(master);
            if(begin) begin--;
    #ifdef MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &endTime);
    #endif
 }
    return NULL;
}

/****************************************************************************/

int main()
{
    printf("Initalization phase 1 started...\n");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    printf("Initalization phase 2 started...\n");
    master = ecrt_request_master(0);
    if (!master)
        return -1;

    printf("Initalization phase 3 started...\n");
    masterDomain = ecrt_master_create_domain(master);
     if (!masterDomain)
        return -1;
    printf("Initalization phase 4 started...\n");
    if (!(sc = ecrt_master_slave_config(master,
                    DRIVE_POS, GOLD_SOLO_DRIVE))) {  
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
            printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc,1,slave_0_syncs))
    {
        fprintf(stderr, "Failed to configure  PDOs!\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to configuring PDOs*\n");
    }
        if (ecrt_domain_reg_pdo_entry_list(masterDomain, masterDomain_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to configuring  PDO entry*\n");
        printf("operation_mode=%d,  ctrl_word=%d,  target_position=%d,  status_word=%d,  mode_display=%d,  current_velocity=%d\n",
                offset.OPERATION_MODE,offset.CONTROL_WORD,offset.TARGET_POSITION,offset.STATUS_WORD,offset.CURRENT_OP_MODE,offset.VELOCITY_COUNTS_PER_SEC);
    }
   /* offset.CONTROL_WORD = ecrt_slave_config_reg_pdo_entry(sc,
            0x6040, 0x00, masterDomain, NULL);
    if (offset.CONTROL_WORD < 0)
        return -1;
    printf("Initalization phase 6 started...\n");
    offset.TARGET_POSITION = ecrt_slave_config_reg_pdo_entry(sc,
            0x607a, 0x00, masterDomain, NULL);
    if (offset.TARGET_POSITION < 0)
        return -1;
    printf("TARGET_POSITION PDO ENTRY REGISTERED...\n");
    offset.TARGET_VELOCITY = ecrt_slave_config_reg_pdo_entry(sc,
            0x60ff, 0x00, masterDomain,NULL);
    if(offset.TARGET_VELOCITY<0)
    return -1;
    printf("TARGET_VELOCITY PDO ENTRY REGISTERED...\n");
    printf("VELOCITY_OFFSET PDO ENTRY REGISTERED...\n");
    offset.STATUS_WORD = ecrt_slave_config_reg_pdo_entry(sc,
            0x6041, 0x00, masterDomain, NULL);
    if (offset.STATUS_WORD < 0)
        return -1;
    printf("STATUS_WORD PDO ENTRY REGISTERED...\n");
    offset.POSITION_ACTUAL_VALUE = ecrt_slave_config_reg_pdo_entry(sc,
            0x6064, 0x00, masterDomain, NULL);
    if (offset.POSITION_ACTUAL_VALUE < 0)
        return -1;
    printf("POSITION_ACTUAL_VALUE PDO ENTRY REGISTERED...\n");
    offset.VELOCITY_COUNTS_PER_SEC = ecrt_slave_config_reg_pdo_entry(sc,
            0x6069, 0x00, masterDomain, NULL);
    if (offset.VELOCITY_COUNTS_PER_SEC < 0)
        return -1;
    printf("VELOCITY_COUNTS_PER_SEC PDO ENTRY REGISTERED...\n");
    offset.TORQUE_ACTUAL_VALUE = ecrt_slave_config_reg_pdo_entry(sc,
            0x6077, 0x00, masterDomain, NULL);
    if (offset.TORQUE_ACTUAL_VALUE < 0)
        return -1;
    printf("TORQUE_ACTUAL_VALUE PDO ENTRY REGISTERED...\n");

    offset.CURRENT_OP_MODE = ecrt_slave_config_reg_pdo_entry(sc,
            0x6061, 0x00, masterDomain, NULL);
    if (offset.TORQUE_ACTUAL_VALUE < 0)
        return -1;
    printf("TORQUE_ACTUAL_VALUE PDO ENTRY REGISTERED...\n");
    
    */


    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, 125000, 0, 0);

    
    printf("Activating master...\n");
    if (ecrt_master_activate(master)){
         printf("Master activation error ! ...\n");
         return -1;
    }

    if(!(SERVO_DRIVE_DOMAIN = ecrt_domain_data(masterDomain)))
    {
        printf("Domain PDO registration error... \n");
        return -1;
    }

        /* Set priority */

    struct sched_param param = {};
    pthread_t cyclicThread;
    pthread_attr_t attr;
    int err;
    
    printf("\nStarting cyclic function.\n");
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i\n.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed\n");
    }


    err = pthread_attr_init(&attr);
        if (err) {
                printf("init pthread attributes failed\n");
                goto out;
        }
 
        /* Set a specific stack size  */
        err = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
        if (err) 
        {
            printf("pthread setstacksize failed\n");
            goto out;
        }
 
        /* Set scheduler policy and priority of pthread */
        err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (err) {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        //param.sched_priority = 80;
        err = pthread_attr_setschedparam(&attr, &param);
        if (err) {
                printf("pthread setschedparam failed\n");
                goto out;
        }
        /* Use scheduling parameters of attr */
        err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (err) 
        {
                printf("pthread setinheritsched failed\n");
                goto out;
        }
 
        /* Create a pthread with specified attributes */    
        err = pthread_create(&cyclicThread, &attr, &cyclic_task, NULL);
        if (err) {
                printf("create pthread failed\n");
                goto out;
        }
 
        /* Join the thread and wait until it is done */
        err = pthread_join(cyclicThread, NULL);
        if (err)
                printf("join pthread failed\n");
    
/*
    pthread_attr_init(&attr);

    pthread_attr_setschedpolicy(&attr,SCHED_FIFO);

    param.sched_priority = 80;

    pthread_attr_setschedparam(&attr,&param);

    err = pthread_create(&cyclicThread,&attr,&cyclic_task,NULL);

    err = pthread_join(cyclicThread,NULL);
*/
    //cyclic_task();

out:
        return err;
        
    return 0;
}

/****************************************************************************/