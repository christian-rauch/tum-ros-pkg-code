

#define OMNICOM_MAGIC_VERSION 1002

/* Data we read from the EtherCAT slaves */
typedef struct omniread {  
	int16_t  magic_version; // magic number to prevent version clashes
	int16_t  torque[4];     // currently not filled in kernel module
	uint32_t pkg_count;     // Set in omnidrive kernel module

	// P0344 == 0x4158: torque actual value (this is what we use)
	// P0508 == 0x41fc: calculated motor torque actual value
	uint16_t status[4];
	uint32_t position[4];		// P0362 -> 0x41CE
	
// NO CHANGES BEFORE THIS LINE //////////////////////////////////////////////////////

	uint32_t angle[4];		// P0391 -> 0x4187

	int32_t actual_velocity[4];     // P0353 -> 0x606c

	int16_t speed_ctrlr_output[4];  // P0356 -> 0x4164
	uint32_t revolutions[4];	// P0392 -> 0x4188
	int16_t torque_actual_value[4]; // P0333 -> 0x414d, Iq actual value. See manual page 13, figure 2. 
	int32_t pos_ctrlr_output[4]; 	// P0351 -> 0x415F, Speed set value. See manual page 16, figure 5. 

	// ethercat states
	int slave_state[4];
	int slave_online[4];
	int slave_operational[4];
	int master_link;
	int master_al_states;
	int master_slaves_responding;
	int working_counter;
	int working_counter_state;
} omniread_t;

/* Data we write to the EtherCAT slaves */
typedef struct omniwrite { 
	int16_t  magic_version;         // magic number to prevent version clashes
	uint32_t target_position[4];    // P0361 -> 0x4169, Torque set value. See manual page 17, figure 6. 
	int16_t target_velocity[4];     
	int16_t torque_set_value[4];    // P0331 -> 0x414b, Skips speed controller. See manual page 12, figure 1. 

	int16_t torque_add_set_value[4]; // P1022 -> 0x43fe, Torque additional set value, see manual page 403 and 13. 
} omniwrite_t;

// realtime interface

void omni_write_data(struct omniwrite data);
struct omniread omni_read_data();

int start_omni_realtime(int max_vel);
void stop_omni_realtime();
