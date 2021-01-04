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

/****************************************************************************/

// Application parameters
#define FREQUENCY  1000                         //Set the frequency in here.
#define CLOCK_TO_USE CLOCK_MONOTONIC      // A nonsettable system-wide clock that represents monotonic time/ \
                                            since—as described by POSIX—"some unspecified point in the \
                                            past".  On Linux, that point corresponds to the number of seconds\
                                            that the system has been running since it was booted.
#define MEASURE_TIMING  1
#define MAX_SAFE_STACK (4096 * 4096)
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)                              // Period mesurements in nanoseconds.
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)                    // Period defined in here based on frequency 1sc/freq

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)
       //Since timespec structure consists of sec and ns addition and difference requires new function
        

        // Timespec structure to nanoseconds function.
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)       

/****************************************************************************/

// EtherCAT 
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *masterDomain = NULL;
static ec_domain_state_t masterDomain_state = {};

const ec_slave_config_t *sc = NULL;
static ec_slave_config_state_t sc_state = {};

/****************************************************************************/

// process data
static unsigned int counter = 1;
static unsigned int blink = 0;
// process data
static uint8_t *LAB1_var_domain = NULL;             //domains for process data excange for individual slaves
static uint8_t *LAB2_var_domain = NULL;

#define LAB_1_SlavePos  0, 1                    // $ ethercat slaves returns slave position based on their connection queue
#define LAB_2_SlavePos  0, 0                    // Physical connection order , if you write the order wrong, PDO entry error will occur.

#define LAB_1 0x0000079a, 0xababa001                // ethercat cstruct will retrun the necessary information for slave adresses and values.          
#define LAB_2 0x0000079a, 0xababa002                // slave vendor id and product code.

// offsets for PDO entries
static uint8_t alarmStatus;             
static uint32_t temperatureStatus;
static uint8_t segments;
static uint16_t potentiometer;
static uint8_t swithces;
static unsigned int sync_ref_counter = 0;

const struct timespec cycletime = {0, PERIOD_NS};       // cycletime settings in ns. 
const struct timespec configSleepTime = {0, 5*NSEC_PER_SEC};
static ec_pdo_entry_reg_t masterDomain_regs[] = {
    {LAB_2_SlavePos,  LAB_2, 0X0005, 0X01, &segments},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X01, &potentiometer},
    {LAB_2_SlavePos,  LAB_2, 0X0006, 0X02, &swithces},
    {LAB_1_SlavePos,  LAB_1, 0X0005, 0X01, &alarmStatus},
    {LAB_1_SlavePos,  LAB_1, 0X0006, 0X01, &temperatureStatus},
    {}
};

/******************************* $ ethercat cstruct returns this info in below ************************************/

/* Master 0, Slave 0, "LAB_2"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa002
 * Revision number: 0x00000001
 */
    // ec_pdo_entry_info_t(index, subindex,bit_length)

    static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
        {0x0005, 0x01, 8}, /* Segments */
        {0x0006, 0x01, 16}, /* Potentiometer */
        {0x0006, 0x02, 8}, /* Swithces */
    };
    // ec_pdo_infot_t(index, number_of_entries,entries)
    static ec_pdo_info_t slave_0_pdos[] = {
        {0x1600, 1, slave_0_pdo_entries + 0},           /* Outputs */
        {0x1a00, 2, slave_0_pdo_entries + 1},           /* Inputs */
    };
    //ec_sync_info_t(index,direction,n_pdos,pdos*, watchdogMode)
    static ec_sync_info_t slave_0_syncs[] = {
        {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},    //OUT is sent from master, input comes from slaves.
        {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
        //{0xff}
    };

    /* Master 0, Slave 1, "LAB_1"
    * Vendor ID:       0x0000079a
    * Product code:    0xababa001
    * Revision number: 0x00000001
    */

    static ec_pdo_entry_info_t slave_1_pdo_entries[] = {
        {0x0005, 0x01, 8}, /* Alarm */
        {0x0006, 0x01, 32}, /* Temperature */
    };

    static ec_pdo_info_t slave_1_pdos[] = {
        {0x1600, 1, slave_1_pdo_entries + 0}, /* Outputs */
        {0x1a00, 1, slave_1_pdo_entries + 1}, /* Inputs */
    };

    static ec_sync_info_t slave_1_syncs[] = {
        {0, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
        {1, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
        {0xff}
    };

/***************************  EOF ethercat cstruct *******************************************/

// timespec consists of seconds and nanoseconds, additin \
    requires external function. We use this functions \
    for time specific measurements /

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
        if (ms.slaves_responding < 2) 
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
            min_period=0xffffffff, min_exec=0xffffffff ,
            jitter=0;
                    
        #endif

    // get current time
clock_gettime(CLOCK_TO_USE, &wakeupTime);
int begin=1;
float tempData=0;
unsigned short potVal=0;
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
        counter = 500;
        // check for master state (optional)
    if(!check_master_state()) 
      {
        printf("Error occured slave state error ; lost connection, master state error\n");
         return -1;
      }

    if (!check_slave_config_states())
      {
        printf("Error occured slave state error ; lost connection, slave config error \n");
        return -1;
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
                printf("Temperature     = %f \nPot Value       = %d\n",
                        tempData,potVal);

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
                blink = !blink;
                tempData = EC_READ_REAL(LAB1_var_domain + temperatureStatus);
                potVal = EC_READ_U16(LAB2_var_domain + potentiometer);
    }

            // write process data
            EC_WRITE_U8(LAB2_var_domain + segments, blink ? 0x0c : 0x03);


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

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

int main(int argc, char **argv)
{
    stack_prefault();
   // ec_slave_config_t *sc;
    ec_slave_config_t *slave_config2;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    masterDomain = ecrt_master_create_domain(master);
     if (!masterDomain)
        return -1;


    if (!(sc = ecrt_master_slave_config(master,
                    LAB_2_SlavePos, LAB_2))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
        if (!(slave_config2 = ecrt_master_slave_config(master,
                    LAB_1_SlavePos, LAB_1))) {  
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    segments = ecrt_slave_config_reg_pdo_entry(sc,
            0x0005, 1, masterDomain, NULL);
    if (segments < 0)
        return -1;

    potentiometer = ecrt_slave_config_reg_pdo_entry(sc,
            0x0006, 0x01, masterDomain, NULL);
    if ( potentiometer < 0)
        return -1;

    //ecrt_slave_config_pdos()
    alarmStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x005, 0x01, masterDomain, NULL);
    if (alarmStatus < 0)
        return -1;
    
    temperatureStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x006, 0x01, masterDomain, NULL);
    if (temperatureStatus < 0)
        return -1;


    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc, 0x0006, PERIOD_NS, 1000, 0, 0);
    ecrt_slave_config_dc(slave_config2, 0x0006, PERIOD_NS, 1000, 0, 0);


    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(LAB2_var_domain = ecrt_domain_data(masterDomain))) {
        return -1;
    }
    if(!(LAB1_var_domain = ecrt_domain_data(masterDomain))) 
    return -1;
 

  /*  printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
*/   
        /* Set priority */

    struct sched_param param = {};
    pthread_t cyclicThread;
    pthread_attr_t attr;
    int err;
    
    printf("\nStarting cyclic function.\n");
        //struct sched_param param = {};
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