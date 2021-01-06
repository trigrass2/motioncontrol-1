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
 /*
  * Adaption to somanet by Frank Jeschke <jeschke@fjes.de>
  *
  * for Synapticon GmbH
  */

/*
 *       Copyright (c) 2014, Synapticon GmbH
 *       All rights reserved.
 *
 *       Redistribution and use in source and binary forms, with or without
 *       modification, are permitted provided that the following conditions are met:
 *
 *       1. Redistributions of source code must retain the above copyright notice, this
 *          list of conditions and the following disclaimer.
 *       2. Redistributions in binary form must reproduce the above copyright notice,
 *          this list of conditions and the following disclaimer in the documentation
 *          and/or other materials provided with the distribution.
 *       3. Execution of this software or parts of it exclusively takes place on hardware
 *          produced by Synapticon GmbH.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *       ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *       DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *       ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *       ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *       (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *       SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *       The views and conclusions contained in the software and documentation are those
 *       of the authors and should not be interpreted as representing official policies,
 *       either expressed or implied, of the Synapticon GmbH.
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdarg.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

#define MAJOR 1
#define MINOR 0

#define MAXDBGLVL  3

// Application parameters
#define FREQUENCY 100
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS  1
#define SDO_ACCESS      1
#define CIA402          1

/****************************************************************************/
#define TEST_BIT(num,N)     (num && (1<<N))
#define BIT_SET(num,N)      (num | (1<<N))
#define BIT_RESET(num,N)    (num & ~(1<<N))

/* application global definitions */
static int g_dbglvl = 1;

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state/* = {}*/;

static ec_domain_t *GOLD_SOLO_DOMAIN = NULL;
static ec_domain_state_t GOLD_SOLO_DOMAIN_state/* = {}*/;

static ec_slave_config_t *sc_data_in = NULL;
static ec_slave_config_state_t sc_data_in_state;
#if 0
static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state/* = {}*/;
#endif

// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data pointer
static uint8_t *GOLD_SOLO_PDO_DOMAIN = NULL;

#define GOLD_SOLO_POS     0, 0
#define GOLD_SOLO_ID     0x0000009a, 0x00030924

// offsets for PDO entries
#ifdef CIA402
static unsigned int c_statusWord; /* status word 8 bit */
static unsigned int off_pdo2_in; /* op modes display 8 bit */
static unsigned int c_positionActualVal; /* position value 32 bit */
static unsigned int off_pdo4_in; /* velocity value 32 bit */
static unsigned int off_pdo5_in; /* torque value 16 bit */

static unsigned int c_controlWord; /* control word 8 bit */
static unsigned int off_pdo2_out; /* op modes 8 bit */
static unsigned int off_pdo3_out; /* target torque 16 bit */
static unsigned int c_targetPosition; /* target position 32 bit */
static unsigned int off_pdo5_out; /* target velocity 32 bit */
#else
static unsigned int c_statusWord;
static unsigned int off_pdo2_in;
static unsigned int c_controlWord;
static unsigned int off_pdo2_out;
#endif

/*
static unsigned int off_ana_in_status;
static unsigned int off_ana_in_value;
static unsigned int off_ana_out;
static unsigned int off_dig_out;
 */

#ifdef CIA402
#define CAN_OD_CONTROL_WORD       0x6040 /* RX; 8 bit */
#define CAN_OD_STATUS_WORD        0x6041 /* TX; 8 bit */
#define CAN_OD_MODES              0x6060 /* RX; 8 bit */
#define CAN_OD_MODES_DISP         0x6061 /* TX; 8 bit */

#define CAN_OD_POS_VALUE          0x6064 /* TX; 32 bit */
#define CAN_OD_POS_TARGET         0x607A /* RX; 32 bit */
#define CAN_OD_VEL_VALUE          0x606C /* TX; 32 bit */
#define CAN_OD_VEL_TARGET         0x60ff /* RX; 32 bit */
#define CAN_OD_TOR_VALUE          0x6077 /* TX; 16 bit */
#define CAN_OD_TOR_TARGET         0x6071 /* RX; 16 bit */

const static ec_pdo_entry_reg_t GOLD_SOLO_DOMAIN_regs[] = {
	/* RX */
	{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_CONTROL_WORD, 0, &c_controlWord},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_MODES,      0, &off_pdo2_out},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_TOR_TARGET, 0, &off_pdo3_out},
	{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_POS_TARGET, 0, &c_targetPosition},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_VEL_TARGET, 0, &off_pdo5_out},
	/* TX */
	{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_STATUS_WORD, 0, &c_statusWord},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_MODES_DISP, 0, &off_pdo2_in},
	{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_POS_VALUE, 0, &c_positionActualVal},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_VEL_VALUE, 0, &off_pdo4_in},
	//{GOLD_SOLO_POS, GOLD_SOLO_ID, CAN_OD_TOR_VALUE, 0, &off_pdo5_in},
    {}
};
#else
const static ec_pdo_entry_reg_t GOLD_SOLO_DOMAIN_regs[] = {
	{GOLD_SOLO_POS, GOLD_SOLO_ID, 0x6000, 1, &c_statusWord},
	{GOLD_SOLO_POS, GOLD_SOLO_ID, 0x6000, 2, &off_pdo2_in},
	{GOLD_SOLO_POS, GOLD_SOLO_ID, 0x7000, 2, &c_controlWord},
	{GOLD_SOLO_POS, GOLD_SOLO_ID, 0x7000, 2, &off_pdo2_out},
	/*
    {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 1, &off_ana_in_status},
    {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 2, &off_ana_in_value},
    {AnaOutSlavePos, Beckhoff_EL4102, 0x3001, 1, &off_ana_out},
    {DigOutSlavePos, Beckhoff_EL2032, 0x3001, 1, &off_dig_out},
    	*/
    {0}
};
#endif

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

/* Master 0, Slave 0
 * Vendor ID:       0x0000009a
 * Product code:    0x00030924
 * Revision number: 0x00010420
 */

ec_pdo_entry_info_t GOLD_SOLO_PDO_ENTRIES[] = {
    {0x607a, 0x00, 32},
    {0x60fe, 0x01, 32},
    {0x6040, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x60fd, 0x00, 32},
    {0x6041, 0x00, 16},
};

ec_pdo_info_t GOLD_SOLO_PDO_INDEXES[] = {
    {0x1600, 3, GOLD_SOLO_PDO_ENTRIES + 0},
    {0x1a00, 3, GOLD_SOLO_PDO_ENTRIES + 3},
};

ec_sync_info_t GOLD_SOLO_SYNCS[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, GOLD_SOLO_PDO_INDEXES + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, GOLD_SOLO_PDO_INDEXES + 1, EC_WD_DISABLE},
    {0xff}
};


static void logmsg(int lvl, const char *format, ...);

/*****************************************************************************/

static ec_sdo_request_t *sdo;

/* additional sdo requests */
static ec_sdo_request_t *request[3];

static ec_sdo_request_t *sdo_download_requests[1]; // one for each object
static unsigned sdoexample;

/*****************************************************************************/

void check_GOLD_SOLO_DOMAIN_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(GOLD_SOLO_DOMAIN, &ds);

    if (ds.working_counter != GOLD_SOLO_DOMAIN_state.working_counter)
        logmsg(1, "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != GOLD_SOLO_DOMAIN_state.wc_state)
        logmsg(1, "Domain1: State %u.\n", ds.wc_state);

    GOLD_SOLO_DOMAIN_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        logmsg(1, "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        logmsg(1, "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        logmsg(1, "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_data_in, &s);

    if (s.al_state != sc_data_in_state.al_state)
        logmsg(1, "AnaIn: State 0x%02X.\n", s.al_state);
    if (s.online != sc_data_in_state.online)
        logmsg(1, "AnaIn: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_data_in_state.operational)
        logmsg(1, "AnaIn: %soperational.\n",
                s.operational ? "" : "Not ");

    sc_data_in_state = s;
}

/*****************************************************************************/

#if SDO_ACCESS
void read_sdo(ec_sdo_request_t *req)
{
    switch (ecrt_sdo_request_state(req)) {
        case EC_REQUEST_UNUSED: // request was not used yet
            ecrt_sdo_request_read(req); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            fprintf(stderr, "SDO still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            logmsg(1, "SDO value read: 0x%X\n",
                    EC_READ_U32(ecrt_sdo_request_data(req)));
            ecrt_sdo_request_read(req); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            fprintf(stderr, "Failed to read SDO!\n");
            ecrt_sdo_request_read(req); // retry reading
            break;
    }
}

void write_sdo(ec_sdo_request_t *req, unsigned data)
{
	EC_WRITE_U32(ecrt_sdo_request_data(req), data&0xffffffff);

	switch (ecrt_sdo_request_state(req)) {
		case EC_REQUEST_UNUSED: // request was not used yet
			ecrt_sdo_request_write(req); // trigger first read
			break;
		case EC_REQUEST_BUSY:
			fprintf(stderr, "SDO write still busy...\n");
			break;
		case EC_REQUEST_SUCCESS:
			logmsg(1, "SDO value written: 0x%X\n", data);
			ecrt_sdo_request_write(req); // trigger next read ???
			break;
		case EC_REQUEST_ERROR:
			fprintf(stderr, "Failed to write SDO!\n");
			ecrt_sdo_request_write(req); // retry writing
			break;
	}
}
#endif

/****************************************************************************/

void cyclic_task()
{
    unsigned int sn_status,sn_position;
    int32_t T_position=1e5;
	/* sync the dc clock of the slaves */
	ecrt_master_sync_slave_clocks(master);

	// receive process data
	ecrt_master_receive(master);
	ecrt_domain_process(GOLD_SOLO_DOMAIN);

	// check process data state (optional)
	check_GOLD_SOLO_DOMAIN_state();

	if (counter) {
		counter--;
	} else { // do this at 1 Hz
		counter = FREQUENCY;

		// calculate new process data
		blink = !blink;

		// check for master state (optional)
		check_master_state();

		// check for islave configuration state(s) (optional)
		check_slave_config_states();

#if SDO_ACCESS
		// read process data SDO
		read_sdo(sdo);
		read_sdo(request[0]);
/*		read_sdo(request[1]);
		read_sdo(request[2]);
*/
	//	write_sdo(sdo_download_requests[0], sdoexample); /* SDO download value to the node */
#endif
	}

	/* Read process data */

	sn_status = EC_READ_U16(GOLD_SOLO_PDO_DOMAIN + c_statusWord);
	//unsigned int sn_modes   = EC_READ_U8(GOLD_SOLO_PDO_DOMAIN + off_pdo2_in);
	sn_position = EC_READ_U32(GOLD_SOLO_PDO_DOMAIN + c_positionActualVal);
	//unsigned int sn_velocity = EC_READ_U32(GOLD_SOLO_PDO_DOMAIN + off_pdo4_in);
	//unsigned int sn_torque = EC_READ_U16(GOLD_SOLO_PDO_DOMAIN + off_pdo5_in);

	printf("Status : 0x%x \nT_position: %d\n",
			sn_status,sn_position);

    


    //DS402 CANOpen over EtherCAT State Machine 
    if((sn_status & 0x004f) == 0x0040 )
    EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x0006);
    if((sn_status & 0x006f) == 0x0021 )
    EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x0007);
    if((sn_status & 0x006f) == 0x0023 )
    EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x000f);    
    if((sn_status & 0x006f) == 0x0027){
        EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x001f);    
        if(TEST_BIT(sn_status,10)){
        T_position *=-1;
        EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x000f);    
        printf("Target reached new target : %d \n",T_position);
        //c_word = 0x3F;
        }       
        EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0x001f); 
    }
    if((sn_status & 0x04f)==0x008){
        printf("ERROR : Motor fault state ... \n"
                        "Resetting Motor state\n");
          EC_WRITE_U16(GOLD_SOLO_PDO_DOMAIN + c_controlWord, 0xFFFF);               
    }
	EC_WRITE_S32(GOLD_SOLO_PDO_DOMAIN + c_targetPosition,T_position);

	// send process data
	ecrt_domain_queue(GOLD_SOLO_DOMAIN);
	ecrt_master_send(master);
	//printf("Wrote %x to slave\n",  blink ? TESTWORD1 : TESTWORD2);
}

/****************************************************************************/

void signal_handler(int signum) {
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
            break;
    }
}

/****************************************************************************/

static void logmsg(int lvl, const char *format, ...)
{
	if (lvl > g_dbglvl)
		return;

	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
}

static inline const char *_basename(const char *prog)
{
	const char *p = prog;
	const char *i = p;
	for (i = p; *i != '\0'; i++) {
		if (*i == '/')
			p = i+1;
	}

	return p;
}

static void printversion(const char *prog)
{
	printf("%s v%d.%d\n", _basename(prog), MAJOR, MINOR);
}

static void printhelp(const char *prog)
{
	printf("Usage: %s [-h] [-v] [-l <level>]\n", _basename(prog));
	printf("\n");
	printf("  -h           print this help and exit\n");
	printf("  -v           print version and exit\n");
	printf("  -l <level>   set log level (0..3)\n");
}

static void cmdline(int argc, char **argv)
{
	int flags, opt;
	int nsecs, tfnd;

	const char *options = "hvl:";

	nsecs = 0;
	tfnd = 0;
	flags = 0;

	while ((opt = getopt(argc, argv, options)) != -1) {
		switch (opt) {
		case 'v':
			printversion(argv[0]);
			exit(0);
			break;

		case 'l':
			g_dbglvl = atoi(optarg);
			if (g_dbglvl<0 || g_dbglvl>MAXDBGLVL) {
				fprintf(stderr, "Error unsuported debug level %d.\n", g_dbglvl);
				exit(1);
			}
			break;

		case 'h':
		default:
			printhelp(argv[0]);
			exit(1);
			break;
		}
	}
}

int main(int argc, char **argv)
{
	cmdline(argc, argv);

    ec_slave_config_t *sc;
    struct sigaction sa;
    struct itimerval tv;

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    GOLD_SOLO_DOMAIN = ecrt_master_create_domain(master);
    if (!GOLD_SOLO_DOMAIN)
        return -1;

    if (!(sc_data_in = ecrt_master_slave_config(
                    master, GOLD_SOLO_POS, GOLD_SOLO_ID))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

#if SDO_ACCESS
    fprintf(stderr, "Creating SDO requests...\n");
    if (!(sdo = ecrt_slave_config_create_sdo_request(sc_data_in, CAN_OD_STATUS_WORD, 0, 2))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }else
    {
        printf("Succesfully requested STATUS_WORD...\n");
    }
    
    ecrt_sdo_request_timeout(sdo, 500); // ms

    if (!(request[0] = ecrt_slave_config_create_sdo_request(sc_data_in, CAN_OD_POS_VALUE, 0, 4))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", CAN_OD_POS_VALUE);
	    return -1;
    }else
    {
        printf("Succesfully requested STATUS_WORD...\n");
    }
    ecrt_sdo_request_timeout(request[0], 500); // ms
/*
    if (!(request[1] = ecrt_slave_config_create_sdo_request(sc_data_in, CAN_OD_VEL_VALUE, 0, 4))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", CAN_OD_VEL_VALUE);
	    return -1;
    }else
    {
        printf("Succesfully requested velocity val...\n");
    }
    ecrt_sdo_request_timeout(request[1], 500); // ms

    if (!(request[2] = ecrt_slave_config_create_sdo_request(sc_data_in, CAN_OD_TOR_VALUE, 0, 2))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", CAN_OD_TOR_VALUE);
	    return -1;
    }else
    {
        printf("Succesfully requested torque value...\n");
    }
    ecrt_sdo_request_timeout(request[2], 500); // ms

    /* register sdo download request */
  /*  if (!(sdo_download_requests[0] = ecrt_slave_config_create_sdo_request(sc_data_in, CAN_OD_MODES, 0,1))) {
	    fprintf(stderr, "Failed to create SDO download request for object 0x%4x\n", CAN_OD_MODES);
	    return -1;
    }else
    {
        printf("Succesfully requested operation mode...\n");
    }
    ecrt_sdo_request_timeout(sdo_download_requests[0], 500); // ms
    */
    /* set the sdoexample to a specific bit muster */
   // sdoexample = 0x22442244;
#endif

#if CONFIGURE_PDOS
    logmsg(1, "Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_data_in, EC_END, GOLD_SOLO_SYNCS)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(
                    master, GOLD_SOLO_POS, GOLD_SOLO_ID))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

#if 0
    if (ecrt_slave_config_pdos(sc, EC_END, el4102_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(
                    master, DigOutSlavePos, Beckhoff_EL2032))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, el2004_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
#endif
#endif

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, GOLD_SOLO_POS /*BusCouplerPos*/, GOLD_SOLO_ID/*Beckhoff_EK1100*/);
    if (!sc)
        return -1;

    if (ecrt_domain_reg_pdo_entry_list(GOLD_SOLO_DOMAIN, GOLD_SOLO_DOMAIN_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    logmsg(1, "Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(GOLD_SOLO_PDO_DOMAIN = ecrt_domain_data(GOLD_SOLO_DOMAIN))) {
        return -1;
    }

#if PRIORITY
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
#endif

    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0)) {
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }

    logmsg(1, "Starting timer...\n");
    tv.it_interval.tv_sec = 0;
    tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    tv.it_value.tv_sec = 0;
    tv.it_value.tv_usec = 1000;
    if (setitimer(ITIMER_REAL, &tv, NULL)) {
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    logmsg(0, "Started.\n");
    while (1) {
        pause();

#if 0
        struct timeval t;
        gettimeofday(&t, NULL);
        printf("%u.%06u\n", (unsigned)t.tv_sec, (unsigned)t.tv_usec);
#endif

        while (sig_alarms != user_alarms) {
            cyclic_task();
            user_alarms++;
        }
    }

    return 0;
}

/****************************************************************************/
