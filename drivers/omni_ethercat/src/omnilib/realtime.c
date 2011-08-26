/*
 * This file is part of the omnimod project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *                    Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <pthread.h>

/****************************************************************************/

#include <igh_eml/ecrt.h>

/****************************************************************************/

#include "realtime.h"  // defines omniread_t, omniwrite_t

/*****************************************************************************/

#define FREQUENCY 1000

/* Optional features */
#define CONFIGURE_PDOS  1
#define EXTERNAL_MEMORY 1
#define SDO_ACCESS      0

/*****************************************************************************/

static ec_master_t *master = NULL;
static ec_master_state_t master_state;// = { };

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state;// = { };

static ec_slave_config_t *sc[4];// = { NULL, NULL, NULL, NULL };
static ec_slave_config_state_t sc_state[4];// = {{}, {}, {}, {}};

static char prevent_set_position = 0;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int exiting = 0;
static pthread_t thread;
static int misses=0;

/*****************************************************************************/

/* Process data */
static uint8_t *domain1_pd;	/* Process data memory */

/* Slave alias, slave position */
#define M0SlavePos 0, 0
#define M1SlavePos 0, 1
#define M2SlavePos 0, 2
#define M3SlavePos 0, 3

/* Slave vendor ID, slave product code */
#define BM3411 0x0000015a, 0x03010001

/* Offsets for 'write' PDO entries */
// static unsigned int off_controlword[4];
static unsigned int off_target_velocity[4];

/* Offsets for 'read' PDO entries */
static unsigned int off_statusword[4];
static unsigned int off_speed_ctrlr_output[4];
static unsigned int off_actual_position[4];
static unsigned int off_actual_velocity[4];

static unsigned int off_torque_actual_value[4];
static unsigned int off_torque_add_value[4];
static unsigned int off_target_position[4];
static unsigned int off_pos_ctrlr_output[4];

/* (alias, position), (VID, PID), PDO entry index, PDO entry subindex, pointer, bit position */
static const ec_pdo_entry_reg_t domain1_regs[] = {

	{M0SlavePos, BM3411, 0x4169, 0, &off_target_position[0], 0},
	{M1SlavePos, BM3411, 0x4169, 0, &off_target_position[1], 0},
	{M2SlavePos, BM3411, 0x4169, 0, &off_target_position[2], 0},
	{M3SlavePos, BM3411, 0x4169, 0, &off_target_position[3], 0},

	{M0SlavePos, BM3411, 0x6042, 0, &off_target_velocity[0], 0},
	{M1SlavePos, BM3411, 0x6042, 0, &off_target_velocity[1], 0},
	{M2SlavePos, BM3411, 0x6042, 0, &off_target_velocity[2], 0},
	{M3SlavePos, BM3411, 0x6042, 0, &off_target_velocity[3], 0},

	/* Torque additional set value, P1022 -> 0x43fe */
	{M0SlavePos, BM3411, 0x43fe, 0, &off_torque_add_value[0], 0},
	{M1SlavePos, BM3411, 0x43fe, 0, &off_torque_add_value[1], 0},
	{M2SlavePos, BM3411, 0x43fe, 0, &off_torque_add_value[2], 0},
	{M3SlavePos, BM3411, 0x43fe, 0, &off_torque_add_value[3], 0},



	{M0SlavePos, BM3411, 0x6041, 0, &off_statusword[0], 0},
	{M1SlavePos, BM3411, 0x6041, 0, &off_statusword[1], 0},
	{M2SlavePos, BM3411, 0x6041, 0, &off_statusword[2], 0},
	{M3SlavePos, BM3411, 0x6041, 0, &off_statusword[3], 0},

	/* P0351 */
 	{M0SlavePos, BM3411, 0x415F, 0, &off_pos_ctrlr_output[0], 0},
 	{M1SlavePos, BM3411, 0x415F, 0, &off_pos_ctrlr_output[1], 0},
 	{M2SlavePos, BM3411, 0x415F, 0, &off_pos_ctrlr_output[2], 0},
 	{M3SlavePos, BM3411, 0x415F, 0, &off_pos_ctrlr_output[3], 0},

	/* P0362 */
 	{M0SlavePos, BM3411, 0x416a, 0, &off_actual_position[0], 0},
 	{M1SlavePos, BM3411, 0x416a, 0, &off_actual_position[1], 0},
 	{M2SlavePos, BM3411, 0x416a, 0, &off_actual_position[2], 0},
 	{M3SlavePos, BM3411, 0x416a, 0, &off_actual_position[3], 0},

	/* P0353 */
	{M0SlavePos, BM3411, 0x606c, 0, &off_actual_velocity[0], 0},
	{M1SlavePos, BM3411, 0x606c, 0, &off_actual_velocity[1], 0},
	{M2SlavePos, BM3411, 0x606c, 0, &off_actual_velocity[2], 0},
	{M3SlavePos, BM3411, 0x606c, 0, &off_actual_velocity[3], 0},
	 
	/* P0333 -> 0x4000 + hex(333) = 0x414d */
	{M0SlavePos, BM3411, 0x414d, 0, &off_torque_actual_value[0], 0},
	{M1SlavePos, BM3411, 0x414d, 0, &off_torque_actual_value[1], 0},
	{M2SlavePos, BM3411, 0x414d, 0, &off_torque_actual_value[2], 0},
	{M3SlavePos, BM3411, 0x414d, 0, &off_torque_actual_value[3], 0},

        {0,0,        0,0,    0,      0, 0,                           0}
};

static unsigned int counter = 0;

static int max_v = 100;

static omniwrite_t tar, tar_buffer;  /* Target velocities */
static omniread_t cur, cur_buffer;    /* Current velocities/torques/positions */

/*****************************************************************************/

/* PDO index, subindex, size in bits */
static ec_pdo_entry_info_t foo_pdo_entries[] = {
	/* Write */
	// {0x6040, 0, 16},	/* control word */
	{0x4169, 0, 32},	/* target position */
	{0x6042, 0, 16},	/* target velocity */
	{0x43fe, 0, 16}, 	/* torque additional set value */

	/* Read */
	{0x6041, 0, 16},	/* statusword */
	{0x415F, 0, 16},	/* pos ctrlr output */
	{0x416a, 0, 32},	/* actual position unsigned */
	{0x606c, 0, 32},	/* actual velocity */
	{0x414d, 0, 16},    /* torque actual value */
};

/* PDO index, #entries, array of entries to map */
static ec_pdo_info_t foo_pdos[] = {
	{0x1600, 3, foo_pdo_entries}, 
	{0x1a00, 5, foo_pdo_entries + 3}, 
};

/* Sync manager index, SM direction, #PDOs, arrays with PDOs to assign */
static ec_sync_info_t foo_syncs[] = {
	{2 /* SM2 */ , EC_DIR_OUTPUT, 1, foo_pdos, EC_WD_DISABLE},
	{3 /* SM3 */ , EC_DIR_INPUT, 1, foo_pdos + 1, EC_WD_DISABLE},
	{0xff        , 0,            0, 0, EC_WD_DISABLE}
};


/*****************************************************************************/


void check_domain1_state(void)
{
	ec_domain_state_t ds;

	ecrt_domain_state(domain1, &ds);

	if (ds.working_counter != domain1_state.working_counter)
		printf("Domain1: WC %u.\n", ds.working_counter);
	if (ds.wc_state != domain1_state.wc_state)
		printf("Domain1: State %u.\n", ds.wc_state);

	domain1_state = ds;
	cur.working_counter = ds.working_counter;
	cur.working_counter_state = ds.wc_state;
}


/*****************************************************************************/


void check_master_state(void)
{
	ec_master_state_t ms;

	ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
		printf("%u slave(s).\n", ms.slaves_responding);
	if (ms.al_states != master_state.al_states)
		printf("AL states: 0x%02X.\n", ms.al_states);
	if (ms.link_up != master_state.link_up)
		printf("Link is %s.\n",
		       ms.link_up ? "up" : "down");

	master_state = ms;
	cur.master_link = ms.link_up;
	cur.master_al_states = ms.al_states;
	cur.master_slaves_responding = ms.slaves_responding;

}

/*****************************************************************************/

void check_slave_config_states(void)
{
	int i;

	ec_slave_config_state_t s;

	for (i = 0; i < 4; i++) {
		ecrt_slave_config_state(sc[i], &s);
		if (s.al_state != sc_state[i].al_state)
			printf("m%d: State 0x%02X.\n", i, s.al_state);
		if (s.online != sc_state[i].online)
			printf("m%d: %s.\n", i,
			       s.online ? "online" : "offline");
		if (s.operational != sc_state[i].operational)
			printf("m%d: %soperational.\n", i,
			       s.operational ? "" : "Not ");
		sc_state[i] = s;

		cur.slave_state[i] = s.al_state;
		cur.slave_online[i] = s.online;
		cur.slave_operational[i] = s.operational;	
	}
}


/*****************************************************************************/

void cyclic_task()
{
	int i;

	/* Receive process data. */
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);

	/* Check process data state (optional). */
	check_domain1_state();

	for (i = 0; i < 4; i++) {
		cur.status[i]         		= EC_READ_U16(domain1_pd + off_statusword[i]);
		cur.speed_ctrlr_output[i]  	= EC_READ_S16(domain1_pd + off_speed_ctrlr_output[i]);
		cur.pos_ctrlr_output[i]  	= EC_READ_S16(domain1_pd + off_pos_ctrlr_output[i]);

		cur.position[i]             = EC_READ_U32(domain1_pd + off_actual_position[i]);
		cur.actual_velocity[i]		= EC_READ_S32(domain1_pd + off_actual_velocity[i]);
		cur.torque_actual_value[i] 	= EC_READ_S16(domain1_pd + off_torque_actual_value[i]); 
	}


    // TODO: factor out these calls
	if (counter) {
		counter--;
	} else {		/* Do this at 1 Hz */
		counter = FREQUENCY;

		/* Check for master state (optional). */
		check_master_state();

		/* Check for slave configuration state(s) (optional). */
		check_slave_config_states();
	}

	if(tar.target_position[0] == 0 && 
	   tar.target_position[1] == 0 && 
	   tar.target_position[2] == 0 && 
	   tar.target_position[3] == 0) { 
		for (i = 0; i < 4; i++) { 
			EC_WRITE_U32(domain1_pd + off_target_position[i], cur.position[i]);
		}
		prevent_set_position = 1; 
	} else { 
		prevent_set_position = 0; 
	}

	/* Write process data. */
	/* Note: You _must_ write something, as this is how the drives sync. */
	for (i = 0; i < 4; i++) { 
		if(!prevent_set_position) { 
			EC_WRITE_U32(domain1_pd + off_target_position[i], tar.target_position[i]);
		}
		EC_WRITE_S16(domain1_pd + off_torque_add_value[i], tar.torque_add_set_value[i]);
		EC_WRITE_S16(domain1_pd + off_target_velocity[i], tar.target_velocity[i]);
	}

	/* Send process data. */
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);

	cur.pkg_count = counter;

}


/*****************************************************************************/


static void enforce_max_velocities(omniwrite_t *t)
{
	int i;

	for (i = 0; i < 4; i++) {
		t->target_velocity[i] =
		  (t->target_velocity[i] > max_v) ? max_v : t->target_velocity[i];
		t->target_velocity[i] =
		  (t->target_velocity[i] < -max_v) ? -max_v : t->target_velocity[i];
	}
}


/*****************************************************************************/

static void stop_motors(void)
{
	int i;

	for (i = 0; i < 4; i++)
		EC_WRITE_S16(domain1_pd + off_target_velocity[i], 0);

	/* Send process data. */
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);

	//usleep(1000);
}


static void timespecInc(struct timespec *tick, int nsec)
{
  tick->tv_nsec += nsec;
  while (tick->tv_nsec >= 1e9)
  {
    tick->tv_nsec -= 1e9;
    tick->tv_sec++;
  }
}


void* realtimeMain(void* udata)
{
  struct timespec tick;
  int period = 1e+6; // 1 ms in nanoseconds

  while(!exiting)
  {
    cyclic_task();

    if(pthread_mutex_trylock(&mutex) == 0)
    {
      tar = tar_buffer;
      cur_buffer = cur;
      pthread_mutex_unlock(&mutex);
    }

    // Compute end of next period
    timespecInc(&tick, period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + before.tv_nsec/1e9) > (tick.tv_sec + tick.tv_nsec/1e9))
    {
      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(&tick, period);

      misses++;
    }
    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  }

  stop_motors();

  
  printf("Releasing master...\n");
  ecrt_release_master(master);

  return 0;
}


/*****************************************************************************/

/* Interface functions */

int start_omni_realtime(int max_vel)
{
	int i;

    max_v = max_vel;

	printf("Init omni...\n");

	/* Zero out the target/current data structs, just in case. */
	memset(&tar, 0, sizeof(tar));
	memset(&cur, 0, sizeof(cur));
	cur.magic_version = OMNICOM_MAGIC_VERSION;

	printf("Starting omni....\n");

	if (!(master = ecrt_request_master(0))) {
		printf( "Requesting master 0 failed!\n");
		goto out_return;
	}

	printf("Registering domain...\n");
	if (!(domain1 = ecrt_master_create_domain(master))) {
		printf( "Domain creation failed!\n");
		goto out_release_master;
	}

	for (i = 0; i < 4; i++) {
		/* master, (slave alias, slave position), (VID, PID) */
		if (!(sc[i] = ecrt_master_slave_config(master, 0, i /* M?SlavePos */, BM3411))) {
			printf(
			       "Failed to get slave configuration for motor %d.\n", i);
			goto out_release_master;
		}
		printf("Configuring PDOs for motor %d...\n", i);
		/* slave config, sync manager index, index of the PDO to assign */
		if (ecrt_slave_config_pdos(sc[i], EC_END, foo_syncs)) {
			printf( "Failed to configure PDOs for motor %d.\n", i);
			goto out_release_master;
		}
	}

	printf("Registering PDO entries...\n");
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		printf( "PDO entry registration failed!\n");
		goto out_release_master;
	}

	printf("Activating master...\n");
	if (ecrt_master_activate(master)) {
		printf( "Failed to activate master!\n");
		goto out_release_master;
	}
	/* Get internal process data for domain. */
	domain1_pd = ecrt_domain_data(domain1);

	printf("Starting cyclic thread.\n");

    pthread_attr_t tattr;
    struct sched_param sparam;
    sparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_init(&tattr);
    pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    pthread_attr_setschedparam(&tattr, &sparam);
    pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);
    
    if(pthread_create(&thread, &tattr, &realtimeMain, 0) != 0) {
      printf("# ERROR: could not create realtime thread\n");
      goto out_release_master;
    }


	printf("Started.\n");

	return 1;

out_release_master:
	printf( "Releasing master...\n");
	ecrt_release_master(master);
out_return:
	printf( "Failed to load. Aborting.\n");
	return 0;
}

/*****************************************************************************/

void stop_omni_realtime(void)
{
	printf("Stopping...\n");


	/* Signal a stop the realtime thread */
    exiting = 1;
    pthread_join(thread, 0);

	/* Now stop all motors. */
	//stop_motors();
	//usleep(100);


	printf("Unloading.\n");
}


void omni_write_data(struct omniwrite data)
{
  pthread_mutex_lock(&mutex);
  tar_buffer = data;
  enforce_max_velocities(&tar_buffer);
  pthread_mutex_unlock(&mutex);
}

struct omniread omni_read_data()
{
  struct omniread data;
  pthread_mutex_lock(&mutex);
  data = cur_buffer;
  pthread_mutex_unlock(&mutex);
  return data;
}
