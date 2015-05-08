#ifndef __LTE_FDD_ENB_MAIN_H__
#define __LTE_FDD_ENB_MAIN_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "liblte_phy.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <map>                           
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
//#include <thread>

#include "liblte_mac.h"
#include "liblte_interface.h"
#include "LTE_file.h"
#include "LTE_fdd_enb_cnfg_db.h"
#include "LTE_fdd_enb_msgq.h"
#include "LTE_fdd_enb_radio.h"
#include "LTE_fdd_enb_user_mgr.h"
#include "LTE_message_queue.h"                              
                              
/*******************************************************************************
                              DEFINES
*******************************************************************************/
#define LTE_FDD_ENB_CURRENT_TTI_MAX (LIBLTE_PHY_SFN_MAX*10 + 9)

/*******************************************************************************
                              FORWARD DECLARATIONS
*******************************************************************************/
using namespace std;

/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/



/*******************************************************************************
                              FUNCTIONS
*******************************************************************************/

// Modified by Chia-Hao Chang
class LTE_fdd_enb_phy
{
public:
    
    static LTE_fdd_enb_phy* get_instance(void);
    static void cleanup(void);

	LTE_fdd_enb_phy(){}
    void start(LTE_fdd_enb_interface *iface);
    void stop(void);
    void construct_sys_info();
    void LTE_fdd_enb_cnfg_db();
    //void get_sys_info(LTE_FDD_ENB_SYS_INFO_STRUCT& copied_sys_info);

    void* process_dl(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf);
    void* process_ul(LTE_FDD_ENB_RADIO_RX_BUF_STRUCT *rx_buf);

    void handle_dl_schedule(void);
    void handle_ul_schedule(void);

    void radio_interface(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf, LTE_FDD_ENB_RADIO_RX_BUF_STRUCT *rx_buf);
    void radio_interface(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf);

    void* operator new(size_t sz){ return malloc(sz); }
    void  operator delete(void* ptr){ return free(ptr); }
    
    /***********************/
    /*   Inline functions  */
    /***********************/
    int  get_sample_rate();
    int  get_N_samps_per_subframe(){ return N_samps_per_subfr; }
    void get_sys_info(LTE_FDD_ENB_SYS_INFO_STRUCT &_sys_info)
    { 
        memcpy(&_sys_info, &sys_info, sizeof(LTE_FDD_ENB_SYS_INFO_STRUCT));
    }
    uint32 get_n_cce(void)
    {
        boost::mutex::scoped_lock lock(sys_info_mutex);
        uint32                    N_cce;
    
        liblte_phy_get_n_cce(phy_struct,
                             liblte_rrc_phich_resource_num[sys_info.mib.phich_config.res],
                             pdcch.N_symbs,
                             sys_info.N_ant,
                             &N_cce);
    
        return(N_cce);
    }
    uint8 bits_2_value_char(uint8  **bits, uint32   N_bits)
    {
        uint32 value = 0;
        uint32 i;
    
        for(i=0; i<N_bits; i++)
        {
            value |= (*bits)[i] << (N_bits-i-1);
        }
        *bits += N_bits;
    
        return(value);
    }

    /***********************/
    /*    Communication    */
    /***********************/
    void handle_mac_msg(LTE_FDD_ENB_MESSAGE_STRUCT *msg);

    LTE_fdd_enb_interface *interface;

    //boost::thread       ul_thread;
    //boost::thread       dl_thread;
    boost::mutex        stream_mutex;


    // intentional public data
    LIBLTE_PHY_SUBFRAME_STRUCT         dl_subframe;

private:
    LIBLTE_PHY_STRUCT *phy_struct;
    static LTE_fdd_enb_phy *instance;

    LTE_FDD_ENB_SYS_INFO_STRUCT        sys_info;
    LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT dl_schedule[10];
    LTE_FDD_ENB_UL_SCHEDULE_MSG_STRUCT ul_schedule[10];
    LIBLTE_PHY_PCFICH_STRUCT           pcfich;
    LIBLTE_PHY_PHICH_STRUCT            phich[10];
    LIBLTE_PHY_PDCCH_STRUCT            pdcch;
    
    LIBLTE_BIT_MSG_STRUCT              dl_rrc_msg;
    uint32                             dl_current_tti;
    uint32                             last_rts_current_tti;
    bool                               late_subfr;
    bool                               started; 

    uint32                             N_samps_per_subfr;
    uint32                             fs;

    boost::mutex                       sys_info_mutex;
    boost::mutex                       dl_sched_mutex;
    boost::mutex                       ul_sched_mutex;

    // Uplink
    LTE_FDD_ENB_PRACH_DECODE_MSG_STRUCT prach_decode;
    LTE_FDD_ENB_PUCCH_DECODE_MSG_STRUCT pucch_decode;
    LTE_FDD_ENB_PUSCH_DECODE_MSG_STRUCT pusch_decode;
    LIBLTE_PHY_SUBFRAME_STRUCT          ul_subframe;
    uint32                              ul_current_tti;
    uint32                              prach_sfn_mod;
    uint32                              prach_subfn_mod;
    uint32                              prach_subfn_check;
    bool                                prach_subfn_zero_allowed;


    std::map<LTE_FDD_ENB_PARAM_ENUM, double> var_map_double;
    std::map<LTE_FDD_ENB_PARAM_ENUM, int64>  var_map_int64;
    std::map<LTE_FDD_ENB_PARAM_ENUM, uint32> var_map_uint32;

    LTE_fdd_enb_msgq                   *mac_comm_msgq;



};

#endif