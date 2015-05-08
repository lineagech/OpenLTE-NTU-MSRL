#line 2 "LTE_fdd_enb_user_mgr.cc" // Make __FILE__ omit the path
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

    File: LTE_fdd_enb_user_mgr.cc

    Description: Contains all the implementations for the LTE FDD eNodeB
                 user manager.

    Revision History
    ----------    -------------    --------------------------------------------
    11/10/2013    Ben Wojtowicz    Created file
    01/18/2014    Ben Wojtowicz    Added level to debug prints.
    05/04/2014    Ben Wojtowicz    Added C-RNTI timeout timers.
    06/15/2014    Ben Wojtowicz    Deleting user on C-RNTI expiration.
    08/03/2014    Ben Wojtowicz    Refactored add_user.

*******************************************************************************/

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "LTE_fdd_enb_user_mgr.h"
#include "LTE_fdd_enb_timer_mgr.h"
#include "liblte_mac.h"
#include <boost/lexical_cast.hpp>

/*******************************************************************************
                              DEFINES
*******************************************************************************/


/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/


/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/

LTE_fdd_enb_user_mgr* LTE_fdd_enb_user_mgr::instance = NULL;
boost::mutex          user_mgr_instance_mutex;

/*******************************************************************************
                              Extern Declaration
*******************************************************************************/

extern uint8 bits_2_value_char(uint8  **bits, uint32   N_bits);

/*******************************************************************************
                              CLASS IMPLEMENTATIONS
*******************************************************************************/

/*******************/
/*    Singleton    */
/*******************/
LTE_fdd_enb_user_mgr* LTE_fdd_enb_user_mgr::get_instance(void)
{
    boost::mutex::scoped_lock lock(user_mgr_instance_mutex);

    if(NULL == instance)
    {
        instance = new LTE_fdd_enb_user_mgr();
    }

    return(instance);
}
void LTE_fdd_enb_user_mgr::cleanup(void)
{
    boost::mutex::scoped_lock lock(user_mgr_instance_mutex);

    if(NULL != instance)
    {
        delete instance;
        instance = NULL;
    }
}

/********************************/
/*    Constructor/Destructor    */
/********************************/
LTE_fdd_enb_user_mgr::LTE_fdd_enb_user_mgr()
{
    next_c_rnti     = LIBLTE_MAC_C_RNTI_START;
    user_scheduling = false;
    num_user        = 0;
    sched_user      = 0;
}
LTE_fdd_enb_user_mgr::~LTE_fdd_enb_user_mgr()
{
}

/****************************/
/*    External Interface    */
/****************************/
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::get_free_c_rnti(uint16 *c_rnti)
{
    boost::mutex::scoped_lock                     lock(c_rnti_mutex);
    std::map<uint16, LTE_fdd_enb_user*>::iterator iter;
    LTE_FDD_ENB_ERROR_ENUM                        err          = LTE_FDD_ENB_ERROR_NO_FREE_C_RNTI;
    uint16                                        start_c_rnti = next_c_rnti;

    do
    {
        iter = c_rnti_map.find(next_c_rnti++);

        if(LIBLTE_MAC_C_RNTI_END < next_c_rnti)
        {
            next_c_rnti = LIBLTE_MAC_C_RNTI_START;
        }
        if(next_c_rnti == start_c_rnti)
        {
            break;
        }
    }while(c_rnti_map.end() != iter);

    if(next_c_rnti != start_c_rnti)
    {
        *c_rnti = next_c_rnti-1;
        err     = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}
void LTE_fdd_enb_user_mgr::assign_c_rnti(uint16            c_rnti,
                                         LTE_fdd_enb_user *user)
{
    boost::mutex::scoped_lock lock(c_rnti_mutex);

    c_rnti_map[c_rnti] = user;
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::free_c_rnti(uint16 c_rnti)
{
    boost::mutex::scoped_lock                     lock(c_rnti_mutex);
    std::map<uint16, LTE_fdd_enb_user*>::iterator iter = c_rnti_map.find(c_rnti);
    LTE_FDD_ENB_ERROR_ENUM                        err  = LTE_FDD_ENB_ERROR_C_RNTI_NOT_FOUND;

    if(c_rnti_map.end() != iter)
    {
        c_rnti_map.erase(iter);
        if((*iter).second->is_id_set())
        {
            (*iter).second->init();
        }else{
            c_rnti_mutex.unlock();
            del_user(c_rnti);
        }
        err = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::add_user(uint16 c_rnti)
{
    LTE_fdd_enb_interface  *interface = LTE_fdd_enb_interface::get_instance();
    LTE_fdd_enb_timer_mgr  *timer_mgr = LTE_fdd_enb_timer_mgr::get_instance();
    LTE_fdd_enb_user       *new_user  = NULL;
    LTE_fdd_enb_timer_cb    timer_expiry_cb(&LTE_fdd_enb_timer_cb_wrapper<LTE_fdd_enb_user_mgr, &LTE_fdd_enb_user_mgr::handle_c_rnti_timer_expiry>, this);
    LTE_FDD_ENB_ERROR_ENUM  err       = LTE_FDD_ENB_ERROR_NONE;
    uint64                  fake_imsi = 0xF000000000000000UL;
    uint32                  timer_id;

    new_user = new LTE_fdd_enb_user(c_rnti);

    if(NULL != new_user)
    {
        fake_imsi += c_rnti;

        // Allocate new user
        user_mutex.lock();
        user_map[fake_imsi] = new_user;
        user_mutex.unlock();

        // Start a C-RNTI reservation timer
        timer_mgr->start_timer(ULLONG_MAX, timer_expiry_cb, &timer_id);
        timer_id_mutex.lock();
        timer_id_map[timer_id] = c_rnti;
        timer_id_mutex.unlock();

        // Assign C-RNTI
        assign_c_rnti(c_rnti, new_user);
        num_user++;

    }else{
        err = LTE_FDD_ENB_ERROR_BAD_ALLOC;
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::find_user(std::string        imsi,
                                                       LTE_fdd_enb_user **user)
{
    boost::mutex::scoped_lock                      lock(user_mutex);
    std::map<uint64, LTE_fdd_enb_user*>::iterator  iter;
    LTE_FDD_ENB_ERROR_ENUM                         err      = LTE_FDD_ENB_ERROR_USER_NOT_FOUND;
    const char                                    *imsi_str = imsi.c_str();
    uint64                                         imsi_num = 0;
    uint32                                         i;

    if(imsi.length() == 15)
    {
        for(i=0; i<15; i++)
        {
            imsi_num *= 10;
            imsi_num += imsi_str[i] - '0';
        }

        iter = user_map.find(imsi_num);

        if(user_map.end() != iter)
        {
            *user = (*iter).second;
            err   = LTE_FDD_ENB_ERROR_NONE;
        }
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::find_user(uint16             c_rnti,
                                                       LTE_fdd_enb_user **user)
{
    boost::mutex::scoped_lock                     lock(c_rnti_mutex);
    std::map<uint16, LTE_fdd_enb_user*>::iterator iter = c_rnti_map.find(c_rnti);
    LTE_FDD_ENB_ERROR_ENUM                        err  = LTE_FDD_ENB_ERROR_USER_NOT_FOUND;

    if(c_rnti_map.end() != iter &&
       NULL             != (*iter).second)
    {
        *user = (*iter).second;
        err   = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::del_user(std::string imsi)
{
    boost::mutex::scoped_lock                      lock(user_mutex);
    std::map<uint64, LTE_fdd_enb_user*>::iterator  iter;
    LTE_FDD_ENB_ERROR_ENUM                         err      = LTE_FDD_ENB_ERROR_USER_NOT_FOUND;
    const char                                    *imsi_str = imsi.c_str();
    uint64                                         imsi_num = 0;
    uint32                                         i;

    if(imsi.length() == 15)
    {
        for(i=0; i<15; i++)
        {
            imsi_num *= 10;
            imsi_num += imsi_str[i] - '0';
        }

        iter = user_map.find(imsi_num);

        if(user_map.end() != iter)
        {
            delete (*iter).second;
            user_map.erase(iter);
            err = LTE_FDD_ENB_ERROR_NONE;
        }
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user_mgr::del_user(uint16 c_rnti)
{
    boost::mutex::scoped_lock                      u_lock(user_mutex);
    boost::mutex::scoped_lock                      c_lock(c_rnti_mutex);
    boost::mutex::scoped_lock                      t_lock(timer_id_mutex);
    std::map<uint64, LTE_fdd_enb_user*>::iterator  u_iter;
    std::map<uint16, LTE_fdd_enb_user*>::iterator  c_iter = c_rnti_map.find(c_rnti);
    std::map<uint32, uint16>::iterator             t_iter;
    LTE_fdd_enb_timer_mgr                         *timer_mgr = LTE_fdd_enb_timer_mgr::get_instance();
    LTE_FDD_ENB_ERROR_ENUM                         err       = LTE_FDD_ENB_ERROR_USER_NOT_FOUND;
    uint64                                         fake_imsi = 0xF000000000000000UL;

    fake_imsi += c_rnti;

    u_iter = user_map.find(fake_imsi);

    if(user_map.end() != u_iter)
    {
        // Delete user class
        delete (*u_iter).second;
        user_map.erase(u_iter);

        // Free the C-RNTI
        if(c_rnti_map.end() != c_iter)
        {
            c_rnti_map.erase(c_iter);
        }

        // Stop the C-RNTI timer
        for(t_iter=timer_id_map.begin(); t_iter!=timer_id_map.end(); t_iter++)
        {
            if((*t_iter).second == c_rnti)
            {
                timer_mgr->stop_timer((*t_iter).first);
                break;
            }
        }

        err = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}

/**********************/
/*    C-RNTI Timer    */
/**********************/
void LTE_fdd_enb_user_mgr::handle_c_rnti_timer_expiry(uint32 timer_id)
{
    LTE_fdd_enb_interface              *interface = LTE_fdd_enb_interface::get_instance();
    boost::mutex::scoped_lock           lock(timer_id_mutex);
    std::map<uint32, uint16>::iterator  iter = timer_id_map.find(timer_id);

    if(timer_id_map.end() != iter)
    {
        interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_INFO,
                                  LTE_FDD_ENB_DEBUG_LEVEL_USER,
                                  __FILE__,
                                  __LINE__,
                                  "C-RNTI allocation timer expiry C-RNTI=%u",
                                  (*iter).second);
        timer_id_map.erase(iter);

        timer_id_mutex.unlock();
        free_c_rnti((*iter).second);
    }
}


// void LTE_fdd_enb_user_mgr::assign_usr_dl_alloc(uint16 c_rnti,
//                                                uint32 rv_idx,
//                                                uint32 **prb,
//                                                uint32 N_bits,
//                                                uint8* msg_bits)
// {
//     LTE_fdd_enb_user* ue; 
//     find_user(c_rnti, &ue);
//     ue->set_dl_msg(N_bits, msg_bits, rv_idx, prb);
// }

/******************************/
/*     System Information     */
/******************************/
void LTE_fdd_enb_user_mgr::update_sys_info(void)
{
    LTE_fdd_enb_phy *phy = LTE_fdd_enb_phy::get_instance();
    std::map<uint16, LTE_fdd_enb_user*>::iterator it;

    for(it=c_rnti_map.begin(); it!=c_rnti_map.end(); it++)
    {
        phy->get_sys_info((it->second)->sys_info);
    }
}

/************************************************/
/*  Scheduling Information, 
    generated by Chia-Hao Chang                 */
/*  Note: Fixed 1.92MHz, one PDCCH per subframe */
/************************************************/
void LTE_fdd_enb_user_mgr::get_ul_sched(LTE_FDD_ENB_UL_SCHEDULE_MSG_STRUCT* ul_sched)
{
    // typedef struct{
    //     LIBLTE_PHY_PDCCH_STRUCT decodes;
    //     uint32                  N_avail_prbs;
    //     uint32                  N_sched_prbs;
    //     uint32                  current_tti;
    //     uint8                   next_prb;
    // }LTE_FDD_ENB_UL_SCHEDULE_MSG_STRUCT;


}
void LTE_fdd_enb_user_mgr::get_dl_sched(LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT* dl_sched)
{
    // typedef struct{
    //     LIBLTE_PHY_PDCCH_STRUCT dl_allocations;
    //     LIBLTE_PHY_PDCCH_STRUCT ul_allocations;
    //     uint32                  N_avail_prbs;
    //     uint32                  N_sched_prbs;
    //     uint32                  current_tti;
    // }LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT;


    // typedef struct{
    //     LIBLTE_PHY_ALLOCATION_STRUCT alloc[LIBLTE_PHY_PDCCH_MAX_ALLOC];
    //     uint32                       N_symbs;
    //     uint32                       N_alloc;
    // }LIBLTE_PHY_PDCCH_STRUCT;
    if(user_scheduling)
    {
        LTE_fdd_enb_user* usr;

        for(uint32 i=0; i<num_user; i++)
        {
            find_user(LIBLTE_MAC_C_RNTI_START+i, &usr);
        
            if(tti_map[usr->get_c_rnti()] == dl_sched->current_tti)
            {
                sched_mutex.lock();
                if(usr->alloc_chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
                {
                    memcpy(&dl_sched->dl_allocations.alloc[dl_sched->dl_allocations.N_alloc], 
                           &usr->dl_msg,
                           sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
                    dl_sched->dl_allocations.N_alloc    += 1;
                    dl_sched->ul_allocations.N_alloc     = 0;
        
                    // report by uplink
                    report_tti[usr->get_c_rnti()]       = dl_sched->current_tti+4;
            
                }else if(usr->alloc_chan_type == LIBLTE_PHY_CHAN_TYPE_ULSCH){
                    memcpy(&dl_sched->ul_allocations.alloc[dl_sched->ul_allocations.N_alloc], 
                           &usr->ul_msg,
                           sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
                    dl_sched->ul_allocations.N_alloc    += 1;
                    dl_sched->dl_allocations.N_alloc     = 0;
                }
            
                //sched_user = (sched_user+1)%num_user;
                sched_mutex.unlock();
            }
        }
    }
}
void LTE_fdd_enb_user_mgr::find_ul_user(uint32 ul_current_tti, uint16* c_rnti)
{
    std::map<uint16, uint32>::iterator it;
    for(it=report_tti.begin(); it!=report_tti.end(); it++)
    {
        if(it->second == ul_current_tti)
        {
            *c_rnti = it->first;
        }
    }
}

void LTE_fdd_enb_user_mgr::set_tti_map(uint16 c_rnti, uint32 sched_tti)
{
    tti_map[c_rnti] = sched_tti;
}

void LTE_fdd_enb_user_mgr::set_dl_sched(uint16 c_rnti, uint32 work_tti, bool flip_ndi, LIBLTE_PHY_CHAN_TYPE_ENUM tran_chan)
{   
    LTE_fdd_enb_user* usr;
    find_user(c_rnti, &usr);

    sched_mutex.lock();
    // Reconfigure user's ul_msg / dl_msg
    if(flip_ndi)
    { // New transmission,
        usr->alloc_chan_type = tran_chan;
        usr->set_new_transmission();
    }else{
      // Retransmission
        usr->set_retransmission();
    }
    tti_map[c_rnti] = work_tti;

    for(int i=0; i<tti_map.size(); i++)
    {
        cerr << i << "-th is "<< tti_map[LIBLTE_MAC_C_RNTI_START+i]<< endl;
    }    


    user_scheduling = true;

    sched_mutex.unlock();
}

bool LTE_fdd_enb_user_mgr::check_dl_sched(uint32 work_tti)
{
    LTE_fdd_enb_user* usr;
    for(uint32 i=0; i<tti_map.size(); i++)
    {
        find_user(LIBLTE_MAC_C_RNTI_START+i, &usr);
        if(tti_map[LIBLTE_MAC_C_RNTI_START+i] == work_tti){
            return false;
        }
        if((tti_map[LIBLTE_MAC_C_RNTI_START+i]+8)==work_tti 
        && usr->alloc_chan_type==LIBLTE_PHY_CHAN_TYPE_ULSCH ){
            return false;
        }
    }
    return true;
}

void LTE_fdd_enb_user_mgr::receive_msg(uint16 c_rnti, LTE_FDD_ENB_PUSCH_DECODE_MSG_STRUCT *pusch_decode)
{
    LTE_fdd_enb_user* usr;
    find_user(c_rnti, &usr);

    usr->uplink_received_msg.push_back(pusch_decode->msg);
}

void LTE_fdd_enb_user_mgr::print_msg(LIBLTE_BIT_MSG_STRUCT *rrc_msg, uint32 rv_idx)
{
    printf("UL LTE Shared Channel :\n");
    printf("\tMessage Decoded, %d rv:\n", rv_idx);
    printf("\t\t");
    uint8* tmp = rrc_msg->msg;
    for(uint32 i=0; i<rrc_msg->N_bits/8; i++)
    {
        printf("%c",bits_2_value_char(&tmp, 8));
    }
    printf("\n");
}

