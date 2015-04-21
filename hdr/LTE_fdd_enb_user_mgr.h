/*******************************************************************************

    Copyright 2013-2014 Ben Wojtowicz

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*******************************************************************************

    File: LTE_fdd_enb_user_mgr.h

    Description: Contains all the definitions for the LTE FDD eNodeB
                 user manager.

    Revision History
    ----------    -------------    --------------------------------------------
    11/09/2013    Ben Wojtowicz    Created file
    05/04/2014    Ben Wojtowicz    Added C-RNTI timeout timers.
    08/03/2014    Ben Wojtowicz    Refactored add_user.

*******************************************************************************/

#ifndef __LTE_FDD_ENB_USER_MGR_H__
#define __LTE_FDD_ENB_USER_MGR_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "LTE_fdd_enb_interface.h"
#include "LTE_fdd_enb_msgq.h"                              
#include "LTE_fdd_enb_user.h"
#include "LTE_fdd_main.h"
#include <boost/thread/mutex.hpp>
#include <string>

/*******************************************************************************
                              DEFINES
*******************************************************************************/


/*******************************************************************************
                              FORWARD DECLARATIONS
*******************************************************************************/


/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/


/*******************************************************************************
                              CLASS DECLARATIONS
*******************************************************************************/

class LTE_fdd_enb_user_mgr
{
public:
    // Singleton
    static LTE_fdd_enb_user_mgr* get_instance(void);
    static void cleanup(void);

    // External interface
    LTE_FDD_ENB_ERROR_ENUM get_free_c_rnti(uint16 *c_rnti);
    void assign_c_rnti(uint16 c_rnti, LTE_fdd_enb_user *user);
    LTE_FDD_ENB_ERROR_ENUM free_c_rnti(uint16 c_rnti);
    LTE_FDD_ENB_ERROR_ENUM add_user(uint16 c_rnti);
    LTE_FDD_ENB_ERROR_ENUM find_user(std::string imsi, LTE_fdd_enb_user **user);
    LTE_FDD_ENB_ERROR_ENUM find_user(uint16 c_rnti, LTE_fdd_enb_user **user);
    LTE_FDD_ENB_ERROR_ENUM del_user(std::string imsi);
    LTE_FDD_ENB_ERROR_ENUM del_user(uint16 c_rnti);

    // user message and allocation manage
    void assign_usr_dl_alloc(uint16 c_rnti,
                             uint32 rv_idx,
                             uint32 **prb,
                             uint32 N_bits,
                             uint8* msg_bits);
    void assign_usr_ul_alloc(uint16 c_rnti);
    void update_sys_info(void);


    // Manage Scheduling Information
    void receive_msg(uint16 c_rnti, LTE_FDD_ENB_PUSCH_DECODE_MSG_STRUCT *pusch_decode);
    void find_ul_user(uint32 ul_current_tti, uint16* c_rnti);
    void set_dl_sched(uint16 c_rnti, uint32 work_tti, bool flip_ndi, LIBLTE_PHY_CHAN_TYPE_ENUM tran_chan);
    void get_dl_sched(LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT* dl_sched);
    void get_ul_sched(LTE_FDD_ENB_UL_SCHEDULE_MSG_STRUCT* ul_sched);

    void set_tti_map(uint16 c_rnti, uint32 sched_tti);
    std::map<uint16, uint32> tti_map;
    std::map<uint16, uint32> report_tti;


    // Print Message
    void print_msg(LIBLTE_BIT_MSG_STRUCT *rrc_msg, uint32 rv_idx);

private:
    // Singleton
    static LTE_fdd_enb_user_mgr *instance;
    LTE_fdd_enb_user_mgr();
    ~LTE_fdd_enb_user_mgr();

    // C-RNTI Timer
    void handle_c_rnti_timer_expiry(uint32 timer_id);

    std::map<uint16, LIBLTE_PHY_CHAN_TYPE_ENUM> chan_map;
    // User storage
    std::map<uint64, LTE_fdd_enb_user*> user_map;
    std::map<uint16, LTE_fdd_enb_user*> c_rnti_map;
    std::map<uint32, uint16>            timer_id_map;
    boost::mutex                        user_mutex;
    boost::mutex                        c_rnti_mutex;
    boost::mutex                        timer_id_mutex;
    boost::mutex                        sched_mutex;
    uint16                              next_c_rnti;
    uint32                              num_user;
    uint32                              sched_user;
    bool                                user_scheduling;
};

#endif /* __LTE_FDD_ENB_USER_MGR_H__ */
