/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to test EtherCAT master protocol using the IgH EtherCAT master userspace library.   
 *  
 *  
 *  IgH EtherCAT master library for Linux is found at the following URL: 
 *  <http://www.etherlab.org/en/ethercat>
 *
 *
 *
 *
 *  2016 Raimarius Delgado
*/
/****************************************************************************/

#include <linux/module.h>
#include <linux/err.h>
// EtherCAT
#include <ecrt.h>
// Xenomai
#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

/*****************************************************************************/
//Module Parameters
#define PFX "Kernel EtherCAT Test: "
unsigned int syncRefCnt =0;
int sanyoServoOp = 0;

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_config = NULL;
static ec_slave_config_state_t sc_config_state = {};

// Xenomai
//static cycles_t t_last_cycle = 0, t_critical;
static RT_TASK TskEcatCtrl;
static RT_SEM SemMaster;
#define ECATCTRL_TASK_MODE	T_FPU|T_CPU(0)
#define ECATCTRL_TASK_STKSZ (4096)
#define ECATCTRL_TASK_PRIORITY	(99) // xeno: 99 , preempt: 80
#define ECATCTRL_TASK_PERIOD	(1000000L)
/*Covertion Macros */
#define RADPS2RPS(x)        (x*(0.1591549430919))
#define RPS2RADPS(x)        (x/(0.1591549430919))
#define RPM2RPS(x)      (x/60)
#define RPS2RPM(x)      (x*60)
/*****************************************************************************/
//SANYO PROTOCOL
#define SANYODENKI_CONTROL_WORD         0x6040,0x00
#define SANYO_SHUTDOWN                  0x0006
#define SANYO_READY                     0x0006
#define SANYO_SWITCH_ON                 0x0007
#define SANYO_SWITCH_OFF                0x0007
#define SANYO_DISABLE_VOLTAGE           0x0000
#define SANYO_QUICK_STOP                0x0002
#define SANYO_DISABLE_OPERATION         0x0007
#define SANYO_ENABLE_OPERATION          0x000F
#define SANYO_FAULT_RESET               0x0080

#define SANYODENKI_STATUS_WORD          0x6041,0x00
#define SANYO_NOT_READY_TO_SWITCH_ON    0x0000
#define SANYO_SWITCH_ON_DISABLED        0x0040
#define SANYO_READY_TO_SWITCH_ON        0x0021
#define SANYO_SWITCH_ON_ENABLED         0x0023
#define SANYO_OPERATION_ENABLED         0x0027
#define SANYO_QUICK_STOP_ACTIVE         0x0007
#define SANYO_FAULT_REACTION_ACTIVE     0x000F
#define SANYO_FAULT                     0x0008

#define SANYODENKI_OPERATION_MODE       0x6060,0x00 
#define SANYO_PROFILE_POSITION          0x01
#define SANYO_PROFILE_VELOCITY          0x03
#define SANYO_PROFILE_TORQUE            0x04
#define SANYO_HOMING                    0x06
#define SANYO_INTERPOLATED_POSITION     0x07
#define SANYO_CYCLIC_POSITION           0x08
#define SANYO_CYCLIC_VELOCITY           0x09
#define SANYO_CYCLIC_TORQUE             0x10

#define SANYODENKI_ACTUAL_POSITION      0x6064, 0x00    //  RO, int32
#define SANYODENKI_ACTUAL_VELOCITY      0x606C, 0x00    //  RO, int32
#define SANYODENKI_TARGET_POSITION      0x607A, 0x00    //  RW, int32
#define SANYODENKI_TARGET_VELOCITY      0x60FF, 0x00    //  RW, int32

#define SANYO_17BIT_ENCRES          131072
#define SANYO_ACTIVATE_WORD         0x0300
#define SANYO_SYNC0_SHIFT           125000
#define SANYOR2_MAXRPM              (3000)

//process data
static uint8_t *domain1_pd;
#define SANYO_SERVO 0x000001b9, 0x00000002 // Vendor ID , Product Code
#define ALIAS_POSITION(x) 	0,x
#define SANYODENKI_SLAVENUM     1
#define _ALLNODE        (-5)

#define SANYO_SERVO_REGS(x) \
    {ALIAS_POSITION(x), SANYO_SERVO, SANYODENKI_CONTROL_WORD , &sanyoCtrlWordOff[x]}, \
    {ALIAS_POSITION(x), SANYO_SERVO, SANYODENKI_TARGET_VELOCITY, &sanyoTargVelOff[x]}, \
    {ALIAS_POSITION(x), SANYO_SERVO, SANYODENKI_STATUS_WORD, &sanyoStatWordOff[x]}, \
    {ALIAS_POSITION(x), SANYO_SERVO, SANYODENKI_ACTUAL_POSITION, &sanyoActlPosOff[x]}, \
    {ALIAS_POSITION(x), SANYO_SERVO, SANYODENKI_ACTUAL_VELOCITY, &sanyoActlVelOff[x]}

static unsigned int sanyoCtrlWordOff[SANYODENKI_SLAVENUM]  =   {0,};
static unsigned int sanyoTargVelOff[SANYODENKI_SLAVENUM]   =   {0,};
static unsigned int sanyoStatWordOff[SANYODENKI_SLAVENUM]  =   {0,};
static unsigned int sanyoActlPosOff[SANYODENKI_SLAVENUM]   =   {0,};
static unsigned int sanyoActlVelOff[SANYODENKI_SLAVENUM]   =   {0,};


ec_pdo_entry_info_t sanyo_pdo_entries[] = {
    {0x6040,0x00,16}, // Controlword 
    {0x60FF,0x00,32}, // Target Velocity 
    {0x6041,0x00,16}, // Statusword 
    {0x6064,0x00,32}, // Position Actual Value 
    {0x606C,0x00,32}, // Velocity Actual Value 
};
ec_pdo_info_t sanyo_pdos[] = {
    {0x1601, 2, sanyo_pdo_entries + 0}, // Tx PDO Mapping 
    {0x1a01, 3, sanyo_pdo_entries + 2}, // Rx PDO Mapping 
};
ec_sync_info_t sanyo_syncs[] = {
    {0, EC_DIR_OUTPUT, 	0, NULL, 		EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 	0, NULL, 		EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 	1, sanyo_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 	1, sanyo_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

const static ec_pdo_entry_reg_t embdDomain_regs[] = {
	SANYO_SERVO_REGS(0),
    {}
};
/*****************************************************************************/
void check_domain1_state(void)
{
    ec_domain_state_t ds;

    rt_sem_p(&SemMaster,TM_INFINITE);
    ecrt_domain_state(domain1, &ds);
    rt_sem_v(&SemMaster);

    if (ds.working_counter != domain1_state.working_counter)
        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}
/*****************************************************************************/
void check_master_state(void)
{
    ec_master_state_t ms;

    rt_sem_p(&SemMaster,TM_INFINITE);
    ecrt_master_state(master, &ms);
    rt_sem_v(&SemMaster);

    if (ms.slaves_responding != master_state.slaves_responding)
        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}
/*****************************************************************************/
void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    rt_sem_p(&SemMaster,TM_INFINITE);
    ecrt_slave_config_state(sc_config, &s);
    rt_sem_v(&SemMaster);

    if (s.al_state != sc_config_state.al_state)
        printk(KERN_INFO PFX "AnaIn: State 0x%02X.\n", s.al_state);
    if (s.online != sc_config_state.online)
        printk(KERN_INFO PFX "AnaIn: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_config_state.operational)
        printk(KERN_INFO PFX "AnaIn: %soperational.\n",
                s.operational ? "" : "Not ");

    sc_config_state = s;
}
/*****************************************************************************/
/*SANYO CANopen State Machine */
/*****************************************************************************/

/*****************************************************************************/
unsigned short sanyoGetStatusVal(unsigned short statusWord)
{
    if((statusWord & 0x4F) == 0x00) {
        statusWord = SANYO_NOT_READY_TO_SWITCH_ON;
    }
    else if((statusWord & 0x4F) == 0x08) {
        statusWord =  SANYO_FAULT;
    }
    else if((statusWord & 0x4F) == 0x40) {
        statusWord =  SANYO_SWITCH_ON_DISABLED;
    }
    else if((statusWord & 0x6F) == 0x27) {
        statusWord =  SANYO_OPERATION_ENABLED;
    }
    else if((statusWord & 0x6F) == 0x23) {
        statusWord =  SANYO_SWITCH_ON_ENABLED;
    }
    else if((statusWord & 0x6F) == 0x21) {
        statusWord =  SANYO_READY_TO_SWITCH_ON;
    }
    else if((statusWord & 0x6F) == 0x07) {
        statusWord =  SANYO_QUICK_STOP_ACTIVE;
    }
    else if((statusWord & 0x4F) == 0x0F) {
        statusWord =  SANYO_FAULT_REACTION_ACTIVE;
    }
    else {
        return 0xFFFF;
    }
    return statusWord;
}
/*****************************************************************************/
void sanyoReady(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_READY);
        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_READY);
    }
}
/*****************************************************************************/
void sanyoSwitchOn(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_SWITCH_ON);
        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_SWITCH_ON);
    }
}
/*****************************************************************************/
void sanyoServoOn(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_ENABLE_OPERATION);
        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_ENABLE_OPERATION);
    }
}
/*****************************************************************************/
void sanyoServoOff(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_DISABLE_OPERATION);

        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_DISABLE_OPERATION);
    }
}
/*****************************************************************************/
void sanyoShutDown(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_SHUTDOWN);

        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_SHUTDOWN);
    }
}
/****************************************************************************/
void sanyoFaultReset(int iNode)
{
    int i;
    
    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[i],SANYO_FAULT_RESET);

        }   
    }
    else{
    
        EC_WRITE_U16(domain1_pd + sanyoCtrlWordOff[iNode],SANYO_FAULT_RESET);
    }
}
/****************************************************************************/
unsigned short sanyoGetStatusWordN(int iNode)
{
    unsigned short statusWord;

    statusWord = sanyoGetStatusVal(EC_READ_U16(domain1_pd + sanyoStatWordOff[iNode]));

    return statusWord;
}
/****************************************************************************/
void sanyoSetVelocity(int iNode, int RevPerMinute)
{
    int i;
    long targetVelocity;

    if (RevPerMinute > SANYOR2_MAXRPM) RevPerMinute = SANYOR2_MAXRPM;
    if (RevPerMinute < -SANYOR2_MAXRPM) RevPerMinute = -SANYOR2_MAXRPM;

    targetVelocity= (long)(SANYO_17BIT_ENCRES * RPM2RPS(RevPerMinute));

    if (iNode == _ALLNODE){
        for(i=0;i<SANYODENKI_SLAVENUM;i++){
            EC_WRITE_U32(domain1_pd + sanyoTargVelOff[i],targetVelocity);
        }   
    }
    else{
        EC_WRITE_U32(domain1_pd + sanyoTargVelOff[iNode],targetVelocity);
    }
}
/****************************************************************************/
/*************************Real Time Task*************************************/
/****************************************************************************/
void run(void *cookie)
{
    RTIME   RtmEcatMasterAppTime;
    int sanyoStatusWord[SANYODENKI_SLAVENUM] = {0,};
    int sanyoPrevStatusWord[SANYODENKI_SLAVENUM] = {0,};
    //int sanyoActualVelR[SANYODENKI_SLAVENUM] = {0,};
    int iSlaveCnt;
    
    while (1) {
        //t_last_cycle = get_cycles();

        // receive process data
        rt_sem_p(&SemMaster,TM_INFINITE);
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        rt_sem_v(&SemMaster);

        // check process data state (optional)
        check_domain1_state();
        check_master_state();
        check_slave_config_states();

        for(iSlaveCnt=0; iSlaveCnt < SANYODENKI_SLAVENUM; ++iSlaveCnt){
            sanyoStatusWord[iSlaveCnt] = sanyoGetStatusWordN(iSlaveCnt);
            //sanyoActualVelR[iSlaveCnt] = sanyoGetActualVelocityN(iSlaveCnt);
            if(sanyoPrevStatusWord[iSlaveCnt] == sanyoStatusWord[iSlaveCnt]){
                continue;
            }
            switch(sanyoStatusWord[iSlaveCnt]){ 
                case SANYO_SWITCH_ON_DISABLED:
                    sanyoReady(iSlaveCnt);
                    break;
                case SANYO_READY_TO_SWITCH_ON:
                    sanyoSwitchOn(iSlaveCnt);
                    break;
                case SANYO_SWITCH_ON_ENABLED:
                    sanyoServoOn(iSlaveCnt);
                    break;
                case SANYO_OPERATION_ENABLED:
                    sanyoServoOp = 1;
                    break;
                case SANYO_FAULT:
                    sanyoFaultReset(iSlaveCnt);
                    break;
                default:
                    break;
            }
            sanyoPrevStatusWord[iSlaveCnt] = sanyoStatusWord[iSlaveCnt]; 
        }
        // write process data
        switch(sanyoServoOp){

            case 1:
                sanyoSetVelocity(_ALLNODE,500);
                break;
            case 2:
                sanyoShutDown(_ALLNODE);
                break;
            default:
                break;
        }
        // write process data
        //EC_WRITE_U32(domain1_pd + sanyoTargVelOff,100000);        

        RtmEcatMasterAppTime = rt_timer_read();
        ecrt_master_application_time(master,(uint64_t)RtmEcatMasterAppTime);
            if (syncRefCnt) {
                syncRefCnt--;
            } else {
                syncRefCnt++; // sync every cycle
                ecrt_master_sync_reference_clock(master);
            }
        ecrt_master_sync_slave_clocks(master);
        
        rt_sem_p(&SemMaster,TM_INFINITE);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        rt_sem_v(&SemMaster);
        rt_task_wait_period(NULL);
    }
}
/****************************************************************************/
int __init init_mod(void)
{
    int ret = -1;
    int i;
    //RTIME tick_period, requested_ticks, now;

//    ec_slave_config_t *sc;

//Xenomai
    rt_sem_create(&SemMaster,"My Semaphore",1,S_FIFO);

//EtherCAT
    printk(KERN_INFO PFX "Starting...\n");
    //t_critical = cpu_khz * 1000 / FREQUENCY - cpu_khz * INHIBIT_TIME / 1000;
    master = ecrt_request_master(0);
    if (!master) {
        ret = -EBUSY;
        printk(KERN_ERR PFX "Requesting master 0 failed!\n");
        goto out_return;
    }

    //ecrt_master_callbacks(master, send_callback, receive_callback, master);

    printk(KERN_INFO PFX "Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain creation failed!\n");
        goto out_release_master;
    }
    for(i=0;i<SANYODENKI_SLAVENUM;i++){
        if (!(sc_config = ecrt_master_slave_config(
                        master, ALIAS_POSITION(i), SANYO_SERVO))) {
            printk(KERN_ERR PFX "Failed to get slave configuration.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Configuring PDOs...\n");
        if (ecrt_slave_config_pdos(sc_config, EC_END, sanyo_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Registering PDO entries...\n");
        if (ecrt_domain_reg_pdo_entry_list(domain1, embdDomain_regs)) {
            printk(KERN_ERR PFX "PDO entry registration failed!\n");
            goto out_release_master;
        }
        if (ecrt_slave_config_sdo8(sc_config,0x6060,0x00,0x09)){
           	printk(KERN_ERR PFX "Failed to config SDOs!\n");
           	goto out_release_master;
    	}
    }
    ecrt_slave_config_dc(sc_config, 0x0300, ECATCTRL_TASK_PERIOD, 10000000, 0, 0);

    printk(KERN_INFO PFX "Activating master...\n");
    if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");
        goto out_release_master;
    }
    // Get internal process data for domain
    domain1_pd = ecrt_domain_data(domain1);

    //Xenomai
    printk(KERN_INFO PFX "Starting cyclic sample thread...\n");
	if(rt_task_create(&TskEcatCtrl,"EtherCAT Control",0,
		ECATCTRL_TASK_PRIORITY,ECATCTRL_TASK_MODE)){
		printk(KERN_ERR PFX "Failed to create Ecat Control Task\n");
		goto out_release_master;
	}
	if(rt_task_set_periodic(&TskEcatCtrl, TM_NOW,rt_timer_ns2ticks(ECATCTRL_TASK_PERIOD))){
		printk(KERN_ERR PFX "Failed to Make Ecat Control Task Periodic\n");
		goto out_stop_task;
	}
	if(rt_task_start(&TskEcatCtrl, &run, NULL)){
		printk(KERN_ERR PFX "Failed to start Ecat Control Task");
		goto out_stop_task;
	}
    printk(KERN_INFO PFX "Initialized.\n");
    return 0;

 out_stop_task:
    rt_task_delete(&TskEcatCtrl);
 //out_stop_timer:
 //   rt_task_delete(&TskEcatCtrl);
 out_release_master:
    printk(KERN_ERR PFX "Releasing master...\n");
    ecrt_release_master(master);
 out_return:
    rt_sem_delete(&SemMaster);
    printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    return ret;
}
/*****************************************************************************/

void __exit cleanup_mod(void)
{
    printk(KERN_INFO PFX "Stopping...\n");
    sanyoServoOp = 2;
    rt_task_delete(&TskEcatCtrl);
    ecrt_release_master(master);
    rt_sem_delete(&SemMaster);
    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Raimarius Delgado <raim223@seoultech.ac.kr>");
MODULE_DESCRIPTION("EtherCAT Xenomai Sample module");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
