/*****************************************************************************
* Example of Panasonic A6B servo drive setup with IgH EtherCAT Master library
* that starts the servo drive in profile velocity mode.
*
* Licence: the same than IgH EtherCAT Master library
*
* ABSTRACT:
*
* This small program shows how to configure a  Panasonic A6B servo drive
* with IgH EtherCAT Master library. It illustrates the flexible PDO mapping
* and the DS402 command register programming in order to start the servo drive
* in profile velocity mode.
*
* Because the  Panasonic A6B servo drive follows the DS402 standard, several
* sections of this example may apply also to other servo drive models.
*
* The code is based on a previous example from IgH EtherCAT Master
*
* Author: Zhou Yonghong
* Date:   2018/04/04
* Compile with:

gcc testbyesm.c -Wall -I /opt/etherlab/include -l ethercat -L /opt/etherlab/lib -o testbyesm

****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/*Application Parameters*/
#define TASK_FREQUENCY          100 /*Hz*/
#define TIMOUT_CLEAR_ERROR  (1*TASK_FREQUENCY)  /*clearing error timeout*/
#define TARGET_VELOCITY         8388608 /*target velocity*/
#define PROFILE_VELOCITY            3   /*Operation mode for 0x6060:0*/

/*****************************************************************************/

/*EtherCAT*/
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_A6B;
static ec_slave_config_state_t sc_A6B_state = {};

/****************************************************************************/

/*Process Data*/
static uint8_t *domain1_pd = NULL;

#define A6BSlavePos         0,0                         /*EtherCAT address on the bus*/

#define Panasonic           0x0000009a,0x00030924   /*Vendor ID, product code*/

/*Offsets for PDO entries*/
static struct{
    unsigned int operation_mode;
    unsigned int ctrl_word;
    unsigned int target_velocity;
    unsigned int status_word;
    unsigned int mode_display;
    unsigned int current_velocity;
//  signed int current_velocity;
}offset;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {A6BSlavePos, Panasonic, 0x6040, 0, &offset.ctrl_word},
    {A6BSlavePos, Panasonic, 0x6060, 0, &offset.operation_mode },
    {A6BSlavePos, Panasonic, 0x60FF, 0, &offset.target_velocity},
    {A6BSlavePos, Panasonic, 0x6041, 0, &offset.status_word},
    {A6BSlavePos, Panasonic, 0x6061, 0, &offset.mode_display},
    {A6BSlavePos, Panasonic, 0x606C, 0, &offset.current_velocity},
    {}
};

static unsigned int counter = 0;
//static  int state = -500;

/***************************************************************************/
/*Config PDOs*/
static ec_pdo_entry_info_t A6B_pdo_entries[] = {
    /*RxPdo 0x1600*/
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8 }, 
    {0x60FF, 0x00, 32},
    /*TxPdo 0x1A00*/
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x606C, 0x00, 32}
};

static ec_pdo_info_t A6B_pdos[] = {
    //RxPdo
    {0x1600, 3, A6B_pdo_entries + 0 },
    //TxPdo
    {0x1A00, 3, A6B_pdo_entries + 3 }
};

static ec_sync_info_t A6B_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    { 2, EC_DIR_OUTPUT, 1, A6B_pdos + 0, EC_WD_DISABLE },
    { 3, EC_DIR_INPUT, 1, A6B_pdos + 1, EC_WD_DISABLE },
    { 0xFF}
};

/**************************************************************************/

/*************************************************************************/



void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    domain1_state = ds;
}

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_A6B, &s);
    if (s.al_state != sc_A6B_state.al_state)
    {
        printf("A6B: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_A6B_state.online)
    {
        printf("A6B: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_A6B_state.operational)
    {
        printf("A6B: %soperational.\n", s.operational ? "" : "Not ");
    }
    sc_A6B_state = s;
}

/*******************************************************************************/

void cyclic_task()
{
    static unsigned int timeout_error = 0;
    static uint16_t command=0x004F;
    static int32_t target_velocity = TARGET_VELOCITY;

    uint16_t    status;
    int8_t      opmode;
    int32_t     current_velocity;
//  int32_t     command_value;

    /*Receive process data*/
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    /*Check process data state(optional)*/
    check_domain1_state();

    counter = TASK_FREQUENCY;
    //Check for master state
    check_master_state();
    //Check for slave configuration state(s)
    check_slave_config_states();
    /*Read inputs*/
    status = EC_READ_U16(domain1_pd + offset.status_word);
    opmode = EC_READ_U8(domain1_pd + offset.mode_display);
    current_velocity = EC_READ_S32(domain1_pd + offset.current_velocity);

    printf("A6B:  act velocity = %d , cmd = 0x%x , status = 0x%x , opmode = 0x%x\n",
            (current_velocity*60)/target_velocity, command, status, opmode);



    //DS402 CANOpen over EtherCAT status machine
    if( (status & command) == 0x0040  )
    {
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x0006 );
        EC_WRITE_S8(domain1_pd + offset.operation_mode, PROFILE_VELOCITY);
        command = 0x006F;
    }

    else if( (status & command) == 0x0021)
    {
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x0007 );
        command = 0x006F;
    }

    else if( (status & command) == 0x0023)
    {
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x000f );
        EC_WRITE_S32(domain1_pd + offset.target_velocity, target_velocity);
        command = 0x006F;
    }
    //operation enabled

    else if( (status & command) == 0x0027)
    {
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x001f);
    }

    /*Send process data*/
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    master = ecrt_request_master(0);
    if (!master)
    {
        exit(EXIT_FAILURE);
    }
    domain1 = ecrt_master_create_domain(master);

    if (!domain1)
    {
        exit(EXIT_FAILURE);
    }
    if (!(sc_A6B = ecrt_master_slave_config(master, A6BSlavePos, Panasonic)))
    {
        fprintf(stderr, "Failed to get slave configuration for A6B!\n");
        exit(EXIT_FAILURE);
    }
//  printf("alias=%d,vid=%d,watchdog_divider=%d",sc_A6B->alias,sc_A6B->position,sc_A6B->watchdog_divider);

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_A6B, EC_END, A6B_syncs))
    {
        fprintf(stderr, "Failed to configure A6B PDOs!\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to configuring A6B PDOs*\n");
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to configuring A6B PDO entry*\n");
        printf("operation_mode=%d,  ctrl_word=%d,  target_velocity=%d,  status_word=%d,  mode_display=%d,  current_velocity=%d\n",
                offset.operation_mode,offset.ctrl_word,offset.target_velocity,offset.status_word,offset.mode_display,offset.current_velocity);
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Master activated*\n");
    }
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        exit(EXIT_FAILURE);
    }

    printf("*It's working now*\n");

    while (1) 
    {
        usleep(100000/TASK_FREQUENCY);
        cyclic_task();
    }
    ecrt_master_deactive(master);
    return EXIT_SUCCESS;
}