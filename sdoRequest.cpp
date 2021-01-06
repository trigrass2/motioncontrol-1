#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdarg.h>
#include <iostream>
/****************************************************************************/

#include "ecrt.h"
#include "ObjectDictionary.h"
/****************************************************************************/

#define MAJOR 1
#define MINOR 0

#define MAXDBGLVL  3

// Application parameters
#define FREQUENCY 1000
#define PRIORITY 1

// Optional features
#define CONFIGURE_PDOS  1
#define SDO_ACCESS      1
#define CIA402          1

/****************************************************************************/

/* application global definitions */
static int g_dbglvl = 1;

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state  = {};

static ec_domain_t *masterDomain = NULL;
static ec_domain_state_t masterDomain_state = {};

static ec_slave_config_t *goldSoloTwitter = NULL;
static ec_slave_config_state_t goldSoloTwitter_state;
#if 0
static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state/* = {}*/;
#endif

// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data pointer
static uint8_t *SERVO_DRIVE_DOMAIN = NULL;

#define DRIVE_POS           0, 0
#define GOLD_SOLO_DRIVE     0x0000009a, 0x00030924

// Offset for PDO Entries;

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


const static ec_pdo_entry_reg_t masterDomain_regs[] = {
    // INPUT PDO MAPPING ;
    {DRIVE_POS,  GOLD_SOLO_DRIVE, positionActualVal     , &offset.POSITION_ACTUAL_VALUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, positonFollowingError , &offset.POSITION_FOLLOWING_ERROR},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, torqueActualValue     , &offset.TORQUE_ACTUAL_VALUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, statusWord            , &offset.STATUS_WORD},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, operationModeDisplay  , &offset.CURRENT_OP_MODE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, positionActualVal     , &offset.POSITION_COUNTS},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, velocityActvalue      , &offset.VELOCITY_COUNTS_PER_SEC},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, currentActualValue    , &offset.CURRENT_ACTUAL_VALUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, errorCode             , &offset.ERROR_CODE},
    //-----------------------------------------------------------------
    // OUTPUT PDO Mapping ; 
    {DRIVE_POS,  GOLD_SOLO_DRIVE, targetPosition    , &offset.TARGET_POSITION},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, targetVelocity    , &offset.TARGET_VELOCITY},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, targetTorque      , &offset.TARGET_TORQUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, maxTorque         , &offset.MAX_TORQUE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, controlWord       , &offset.CONTROL_WORD},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, operationMode     , &offset.OPERATION_MODE},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, maxCurrent        , &offset.MAX_CURRENT},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, profileDeceleration, &offset.PROFILE_DECELERATION},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, profileAcceleration, &offset.PROFILE_ACCELERATION},
    {DRIVE_POS,  GOLD_SOLO_DRIVE, velocityOffset     , &offset.VELOCITY_OFFSET}, 
    {0}
};

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

#if CONFIGURE_PDOS

#ifdef CIA402
/* Master 0, Slave 0, "Synapticon-ECAT"
 * Vendor ID:       0x000022d2
 * Product code:    0x00000201
 * Revision number: 0x0a000002
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* control word */
    {0x6060, 0x00, 8}, /* operating modes */
    {0x6071, 0x00, 16}, /* torque target */
    {0x607a, 0x00, 32}, /* position target */
    {0x60ff, 0x00, 32}, /* velocity target */
    {0x6041, 0x00, 16}, /* status */
    {0x6061, 0x00, 8}, /* modes display */
    {0x6064, 0x00, 32}, /* position value */
    {0x606c, 0x00, 32}, /* velocity value */
    {0x6077, 0x00, 16}, /* torque value */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 5, slave_0_pdo_entries + 0}, /* Rx PDO Mapping */
    {0x1a00, 5, slave_0_pdo_entries + 5}, /* Tx PDO Mapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

#else

/* Master 0, Slave 0, "Synapticon-ECAT"
 * Vendor ID:       0x000022d2
 * Product code:    0x00000201
 * Revision number: 0x0a000002
 */

//static ec_pdo_entry_info_t somanet_pdo_entries[] = { };
static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* ECAT Out1 */
    {0x7000, 0x02, 16}, /* ECAT Out2 */
    {0x6000, 0x01, 16}, /* ECAT In1 */
    {0x6000, 0x02, 16}, /* ECAT In2 */
};

//static ec_pdo_info_t somanet_pdos[] = {};
static ec_pdo_info_t slave_0_pdos[] = {
    {0x1a00, 2, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1600, 2, slave_0_pdo_entries + 2}, /* Inputs */
};

//static ec_sync_info_t somanet_syncs[] = {};
/* this configures the sync manager entries */
static ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
#endif


#endif

static void logmsg(int lvl, const char *format, ...);

/*****************************************************************************/

#if SDO_ACCESS
static ec_sdo_request_t *sdo;

/* additional sdo requests */
static ec_sdo_request_t *request[3];

static ec_sdo_request_t *sdo_download_requests[1]; // one for each object
static unsigned sdoexample;
#endif

/*****************************************************************************/

void check_masterDomain_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(masterDomain, &ds);

    if (ds.working_counter != masterDomain_state.working_counter)
        logmsg(1, "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != masterDomain_state.wc_state)
        logmsg(1, "Domain1: State %u.\n", ds.wc_state);

    masterDomain_state = ds;
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

    ecrt_slave_config_state(goldSoloTwitter, &s);

    if (s.al_state != goldSoloTwitter_state.al_state)
        logmsg(1, "AnaIn: State 0x%02X.\n", s.al_state);
    if (s.online != goldSoloTwitter_state.online)
        logmsg(1, "AnaIn: %s.\n", s.online ? "online" : "offline");
    if (s.operational != goldSoloTwitter_state.operational)
        logmsg(1, "AnaIn: %soperational.\n",
                s.operational ? "" : "Not ");

    goldSoloTwitter_state = s;
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
	/* sync the dc clock of the slaves */
	ecrt_master_sync_slave_clocks(master);

	// receive process data
	ecrt_master_receive(master);
	ecrt_domain_process(masterDomain);

	// check process data state (optional)
	check_masterDomain_state();

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

#if 1
		// read process data SDO
		read_sdo(sdo);
		read_sdo(request[0]);
		read_sdo(request[1]);
		read_sdo(request[2]);

		write_sdo(sdo_download_requests[0], sdoexample); /* SDO download value to the node */
#endif
	}

	/* Read process data */
	unsigned int sn_status = EC_READ_U16(SERVO_DRIVE_DOMAIN     + offset.STATUS_WORD);
	unsigned int sn_modes = EC_READ_U8(SERVO_DRIVE_DOMAIN       + offset.CURRENT_OP_MODE);
	unsigned int sn_position = EC_READ_U32(SERVO_DRIVE_DOMAIN   + offset.POSITION_ACTUAL_VALUE);
	unsigned int sn_velocity = EC_READ_U32(SERVO_DRIVE_DOMAIN   + offset.POSITION_ACTUAL_VALUE);
	unsigned int sn_torque = EC_READ_U16(SERVO_DRIVE_DOMAIN     + offset.TORQUE_ACTUAL_VALUE);

	logmsg(2, "[REC] 0x%4x 0x%4x 0x%8x 0x%8x 0x%4x\n",
			sn_status, sn_modes,
			sn_position, sn_velocity, sn_torque);

#if 0
    // read process data
    printf("AnaIn: state %u value %u\n",
            EC_READ_U8(SERVO_DRIVE_DOMAIN + off_ana_in_status),
            EC_READ_U16(SERVO_DRIVE_DOMAIN + off_ana_in_value));
#endif

    // write process data
    //EC_WRITE_U8(SERVO_DRIVE_DOMAIN + off_dig_out, blink ? 0x06 : 0x09);

	// send process data
	ecrt_domain_queue(masterDomain);
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
    else
    {
        std::cout << "Master requested.. \n";
    }
    
    masterDomain = ecrt_master_create_domain(master);
    if (!masterDomain)
        return -1;
    else
    {
        std::cout << "Master domain created... \n";
    }
    

    if (!(goldSoloTwitter = ecrt_master_slave_config(
                    master, DRIVE_POS, GOLD_SOLO_DRIVE))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    else
    {
        std::cout << "Slave configuration founded...\n";
    }
    

#if SDO_ACCESS
    fprintf(stderr, "Creating SDO requests...\n");
    if (!(sdo = ecrt_slave_config_create_sdo_request(goldSoloTwitter, statusWord, 2))) {
        fprintf(stderr, "Failed to create SDO request.\n");
        return -1;
    }
    ecrt_sdo_request_timeout(sdo, 500); // ms

    if (!(request[0] = ecrt_slave_config_create_sdo_request(goldSoloTwitter, positionActualVal, 4))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", positionActualVal);
	    return -1;
    }
    ecrt_sdo_request_timeout(request[0], 500); // ms

    if (!(request[1] = ecrt_slave_config_create_sdo_request(goldSoloTwitter, velocityActvalue, 4))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", velocityActvalue);
	    return -1;
    }
    ecrt_sdo_request_timeout(request[1], 500); // ms

    if (!(request[2] = ecrt_slave_config_create_sdo_request(goldSoloTwitter, torqueActualValue, 2))) {
	    fprintf(stderr, "Failed to create SDO request for object 0x%4x\n", torqueActualValue);
	    return -1;
    }
    ecrt_sdo_request_timeout(request[2], 500); // ms

    /* register sdo download request */
    if (!(sdo_download_requests[0] = ecrt_slave_config_create_sdo_request(goldSoloTwitter, operationModeDisplay,1))) {
	    fprintf(stderr, "Failed to create SDO download request for object 0x%4x\n", operationModeDisplay);
	    return -1;
    }
    ecrt_sdo_request_timeout(sdo_download_requests[0], 500); // ms

    /* set the sdoexample to a specific bit muster */
    sdoexample = 0x22442244;
#endif

#if CONFIGURE_PDOS

    if (!(sc = ecrt_master_slave_config(
                master, DRIVE_POS, GOLD_SOLO_DRIVE))) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
    
    logmsg(1, "Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(goldSoloTwitter, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
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
    sc = ecrt_master_slave_config(master, DRIVE_POS, GOLD_SOLO_DRIVE/*Beckhoff_EK1100*/);
    if (!sc)
        return -1;

    if (ecrt_domain_reg_pdo_entry_list(masterDomain, masterDomain_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    logmsg(1, "Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(SERVO_DRIVE_DOMAIN = ecrt_domain_data(masterDomain))) {
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
}

/****************************************************************************/
