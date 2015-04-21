#line 2 "LTE_fdd_enb_user.cc" // Make __FILE__ omit the path
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

    File: LTE_fdd_enb_user.cc

    Description: Contains all the implementations for the LTE FDD eNodeB
                 user class.

    Revision History
    ----------    -------------    --------------------------------------------
    11/10/2013    Ben Wojtowicz    Created file
    05/04/2014    Ben Wojtowicz    Added radio bearer support.
    06/15/2014    Ben Wojtowicz    Added initialize routine.
    08/03/2014    Ben Wojtowicz    Refactored user identities.
    09/03/2014    Ben Wojtowicz    Added ciphering and integrity algorithm
                                   storing.

*******************************************************************************/

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "LTE_fdd_enb_user.h"
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


/*******************************************************************************
                              CLASS IMPLEMENTATIONS
*******************************************************************************/

/********************************/
/*    Constructor/Destructor    */
/********************************/
LTE_fdd_enb_user::LTE_fdd_enb_user(uint16 _c_rnti)
{
    uint32 i;

    // Identity
    id_set       = false;
    temp_id      = 0;
    c_rnti       = _c_rnti;
    c_rnti_set   = true;
    auth_vec_set = false;

    // Capabilities
    for(i=0; i<8; i++)
    {
        eea_support[i] = false;
        eia_support[i] = false;
        uea_support[i] = false;
        uia_support[i] = false;
        gea_support[i] = false;
    }
    uea_set = false;
    uia_set = false;
    gea_set = false;

    // Radio Bearers
    /*
    srb0 = new LTE_fdd_enb_rb(LTE_FDD_ENB_RB_SRB0, this);
    srb1 = NULL;
    srb2 = NULL;
    for(i=0; i<8; i++)
    {
        drb[i] = NULL;
    }
    */
    // MAC
    dl_ndi = false;
    ul_ndi = false;

    // Generic
    delete_at_idle = false;

    // User's message
    Msg_N_Bits     = 16;
    tx_information = 'A';

}
LTE_fdd_enb_user::~LTE_fdd_enb_user()
{
    uint32 i;

    // Radio Bearers
    /*
    for(i=0; i<8; i++)
    {
        delete drb[i];
    }
    delete srb2;
    delete srb1;
    delete srb0;
    */
}

/********************/
/*    Initialize    */
/********************/
void LTE_fdd_enb_user::init(void)
{
    uint32 i;

    // Radio Bearers
    /*
    for(i=0; i<8; i++)
    {
        delete drb[i];
    }
    delete srb2;
    delete srb1;
    srb0->set_mme_procedure(LTE_FDD_ENB_MME_PROC_IDLE);
    srb0->set_mme_state(LTE_FDD_ENB_MME_STATE_IDLE);
    srb0->set_rrc_procedure(LTE_FDD_ENB_RRC_PROC_IDLE);
    srb0->set_rrc_state(LTE_FDD_ENB_RRC_STATE_IDLE);
    */

    // MAC
    dl_ndi = false;
    ul_ndi = false;
}

/******************/
/*    Identity    */
/******************/
void LTE_fdd_enb_user::set_id(LTE_FDD_ENB_USER_ID_STRUCT *identity)
{
    memcpy(&id, identity, sizeof(LTE_FDD_ENB_USER_ID_STRUCT));
    id_set = true;
}
LTE_FDD_ENB_USER_ID_STRUCT* LTE_fdd_enb_user::get_id(void)
{
    return(&id);
}
bool LTE_fdd_enb_user::is_id_set(void)
{
    return(id_set);
}
void LTE_fdd_enb_user::set_temp_id(uint64 id)
{
    temp_id = id;
}
uint64 LTE_fdd_enb_user::get_temp_id(void)
{
    return(temp_id);
}
std::string LTE_fdd_enb_user::get_imsi_str(void)
{
    return(boost::lexical_cast<std::string>(id.imsi));
}
uint64 LTE_fdd_enb_user::get_imsi_num(void)
{
    return(id.imsi);
}
std::string LTE_fdd_enb_user::get_imei_str(void)
{
    return(boost::lexical_cast<std::string>(id.imei));
}
uint64 LTE_fdd_enb_user::get_imei_num(void)
{
    return(id.imei);
}
void LTE_fdd_enb_user::set_c_rnti(uint16 _c_rnti)
{
    c_rnti     = _c_rnti;
    c_rnti_set = true;
}
uint16 LTE_fdd_enb_user::get_c_rnti(void)
{
    return(c_rnti);
}
bool LTE_fdd_enb_user::is_c_rnti_set(void)
{
    return(c_rnti_set);
}
void LTE_fdd_enb_user::set_auth_vec(LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT *av)
{
    memcpy(&auth_vec, av, sizeof(LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT));
    auth_vec_set = true;
}
LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT* LTE_fdd_enb_user::get_auth_vec(void)
{
    return(&auth_vec);
}
void LTE_fdd_enb_user::increment_nas_count_dl(void)
{
    if(auth_vec_set)
    {
        auth_vec.nas_count_dl++;
    }
}
void LTE_fdd_enb_user::increment_nas_count_ul(void)
{
    if(auth_vec_set)
    {
        auth_vec.nas_count_ul++;
    }
}
bool LTE_fdd_enb_user::is_auth_vec_set(void)
{
    return(auth_vec_set);
}

/**********************/
/*    Capabilities    */
/**********************/
void LTE_fdd_enb_user::set_eea_support(uint8 eea,
                                       bool  support)
{
    eea_support[eea] = support;
}
bool LTE_fdd_enb_user::get_eea_support(uint8 eea)
{
    return(eea_support[eea]);
}
void LTE_fdd_enb_user::set_eia_support(uint8 eia,
                                       bool  support)
{
    eia_support[eia] = support;
}
bool LTE_fdd_enb_user::get_eia_support(uint8 eia)
{
    return(eia_support[eia]);
}
void LTE_fdd_enb_user::set_uea_support(uint8 uea,
                                       bool  support)
{
    uea_support[uea] = support;
    uea_set          = true;
}
bool LTE_fdd_enb_user::get_uea_support(uint8 uea)
{
    return(uea_support[uea]);
}
bool LTE_fdd_enb_user::is_uea_set(void)
{
    return(uea_set);
}
void LTE_fdd_enb_user::set_uia_support(uint8 uia,
                                       bool  support)
{
    uia_support[uia] = support;
    uia_set          = true;
}
bool LTE_fdd_enb_user::get_uia_support(uint8 uia)
{
    return(uia_support[uia]);
}
bool LTE_fdd_enb_user::is_uia_set(void)
{
    return(uia_set);
}
void LTE_fdd_enb_user::set_gea_support(uint8 gea,
                                       bool  support)
{
    gea_support[gea] = support;
    gea_set          = true;
}
bool LTE_fdd_enb_user::get_gea_support(uint8 gea)
{
    return(gea_support[gea]);
}
bool LTE_fdd_enb_user::is_gea_set(void)
{
    return(gea_set);
}

/***********************/
/*    Radio Bearers    */
/***********************/
/*
void LTE_fdd_enb_user::get_srb0(LTE_fdd_enb_rb **rb)
{
    *rb = srb0;
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::setup_srb1(LTE_fdd_enb_rb **rb)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_ALREADY_SETUP;

    if(NULL == srb1)
    {
        srb1 = new LTE_fdd_enb_rb(LTE_FDD_ENB_RB_SRB1, this);
        err  = LTE_FDD_ENB_ERROR_NONE;
    }
    *rb = srb1;

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::teardown_srb1(void)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_NOT_SETUP;

    if(NULL != srb1)
    {
        delete srb1;
        err = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::get_srb1(LTE_fdd_enb_rb **rb)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_NOT_SETUP;

    if(NULL != srb1)
    {
        err = LTE_FDD_ENB_ERROR_NONE;
    }
    *rb = srb1;

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::setup_srb2(LTE_fdd_enb_rb **rb)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_ALREADY_SETUP;

    if(NULL == srb2)
    {
        srb2 = new LTE_fdd_enb_rb(LTE_FDD_ENB_RB_SRB2, this);
        err  = LTE_FDD_ENB_ERROR_NONE;
    }
    *rb = srb2;

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::teardown_srb2(void)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_NOT_SETUP;

    if(NULL != srb2)
    {
        delete srb2;
        err = LTE_FDD_ENB_ERROR_NONE;
    }

    return(err);
}
LTE_FDD_ENB_ERROR_ENUM LTE_fdd_enb_user::get_srb2(LTE_fdd_enb_rb **rb)
{
    LTE_FDD_ENB_ERROR_ENUM err = LTE_FDD_ENB_ERROR_RB_NOT_SETUP;

    if(NULL != srb2)
    {
        err = LTE_FDD_ENB_ERROR_NONE;
    }
    *rb = srb2;

    return(err);
}
*/
/*************/
/*    MAC    */
/*************/
bool LTE_fdd_enb_user::get_dl_ndi(void)
{
    return(dl_ndi);
}
void LTE_fdd_enb_user::flip_dl_ndi(void)
{
    dl_ndi ^= 1;
}
bool LTE_fdd_enb_user::get_ul_ndi(void)
{
    return(ul_ndi);
}
void LTE_fdd_enb_user::flip_ul_ndi(void)
{
    ul_ndi ^= 1;
}

/*****************/
/*    Generic    */
/*****************/
void LTE_fdd_enb_user::set_delete_at_idle(bool dai)
{
    delete_at_idle = dai;
}
bool LTE_fdd_enb_user::get_delete_at_idle(void)
{
    return(delete_at_idle);
}

void LTE_fdd_enb_user::set_dl_msg(uint32 N_bits, uint8* msg_bits, uint32 rv_idx)
{
    LIBLTE_BIT_MSG_STRUCT   msg_bits_struct;
    uint32                  tbs;
    uint32                  N_prb;
    uint32                  tmp_ndi;
    uint32                  i;
    uint32                  last_prb = 0;

    msg_bits_struct.N_bits = N_bits;
    memcpy(msg_bits_struct.msg, msg_bits, sizeof(uint8)*N_bits);
    
    liblte_phy_get_tbs_and_n_prb_for_dl(msg_bits_struct.N_bits,
                                        sys_info.N_rb_dl,
                                        0, // fixed by Chia-Hao Chang, 36.213 table 7.1.7.1-1
                                        &tbs,
                                        &N_prb);
    for(i=0; i<N_prb; i++)
    {
        Msg_Alloc_PRB[0][i] = last_prb;
        Msg_Alloc_PRB[1][i] = last_prb++;
    }

    set_allocation(&msg_bits_struct,
                   LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY, 
                   LIBLTE_PHY_MODULATION_TYPE_QPSK,
                   LIBLTE_PHY_CHAN_TYPE_DLSCH,
                   tbs,
                   rv_idx, // rv_idx
                   N_prb,
                   Msg_Alloc_PRB,
                   1,   // N_codewords
                   1,   // N_layers
                   1,   // tx_mode
                   c_rnti,
                   0,   // mcs
                   0,   // tpc
                   dl_ndi);
}
void LTE_fdd_enb_user::set_ul_msg(uint32 N_bits)
{
    uint32                  i;
    uint8                   mcs;
    uint32                  tbs;
    uint32                  N_prb;
    uint32                  last_prb = 1;

    liblte_phy_get_tbs_mcs_and_n_prb_for_ul(N_bits,
                                            sys_info.N_rb_dl,
                                            &tbs,
                                            &mcs,
                                            &N_prb);

    for(i=0; i<N_prb; i++)
    {
        Msg_Alloc_PRB[0][i] = last_prb;
        Msg_Alloc_PRB[1][i] = last_prb++;
    }
    set_allocation(NULL,
                   LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY,  // fixed
                   LIBLTE_PHY_MODULATION_TYPE_QPSK,         // fixed
                   LIBLTE_PHY_CHAN_TYPE_ULSCH,              // fixed
                   tbs,
                   0,                                       // rv_idx
                   N_prb,
                   Msg_Alloc_PRB,
                   1,                                       // N_codewords
                   1,                                       // N_layers
                   1,                                       // tx_mode
                   c_rnti,
                   0,                                       // mcs
                   0,                                       // tpc
                   ul_ndi);
}
void LTE_fdd_enb_user::set_allocation(LIBLTE_BIT_MSG_STRUCT*          msg_bits,
                                      LIBLTE_PHY_PRE_CODER_TYPE_ENUM  pre_coder_type,
                                      LIBLTE_PHY_MODULATION_TYPE_ENUM mod_type,
                                      LIBLTE_PHY_CHAN_TYPE_ENUM       chan_type,
                                      uint32                          tbs,
                                      uint32                          rv_idx,
                                      uint32                          N_prb,
                                      uint32                          prb[][6],
                                      uint32                          N_codewords,
                                      uint32                          N_layers,
                                      uint32                          tx_mode,
                                      uint16                          rnti,
                                      uint8                           mcs,
                                      uint8                           tpc,
                                      bool                            ndi)
{
    if(chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
    {
        memcpy(&dl_msg.msg, msg_bits, sizeof(LIBLTE_BIT_MSG_STRUCT));
        dl_msg.pre_coder_type = pre_coder_type;
        dl_msg.mod_type       = mod_type;
        dl_msg.chan_type      = chan_type;
        dl_msg.tbs            = tbs;
        dl_msg.rv_idx         = rv_idx;
        dl_msg.N_prb          = N_prb;
        dl_msg.N_codewords    = N_codewords;
        dl_msg.N_layers       = N_layers;
        dl_msg.tx_mode        = tx_mode;
        dl_msg.rnti           = rnti;
        dl_msg.mcs            = mcs;
        dl_msg.tpc            = tpc; 
        dl_msg.ndi            = ndi;
    
        #pragma omp parallel for
        for(int i=0; i<N_prb; i++)
        {
            dl_msg.prb[0][i] = prb[0][i];
            dl_msg.prb[1][i] = prb[1][i];
        }
    }else{
        ul_msg.pre_coder_type = pre_coder_type;
        ul_msg.mod_type       = mod_type;
        ul_msg.chan_type      = chan_type;
        ul_msg.tbs            = tbs;
        ul_msg.rv_idx         = rv_idx;
        ul_msg.N_prb          = N_prb;
        ul_msg.N_codewords    = N_codewords;
        ul_msg.N_layers       = N_layers;
        ul_msg.tx_mode        = tx_mode;
        ul_msg.rnti           = rnti;
        ul_msg.mcs            = mcs;
        ul_msg.tpc            = tpc; 
        ul_msg.ndi            = ndi;
    
        #pragma omp parallel for
        for(uint32 i=0; i<N_prb; i++)
        {
            ul_msg.prb[0][i] = prb[0][i];
            ul_msg.prb[1][i] = prb[1][i];
        }
    }
}
void LTE_fdd_enb_user::set_new_transmission()
{
    Redundancy_Version_Idx = 0;
    if(this->alloc_chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
    {   
        this->get_information_bits();
        Msg_N_Bits          = 16;  // fixed by Chia-Hao Chang
        this->flip_dl_ndi();
        set_dl_msg(Msg_N_Bits, Msg_Bits, Redundancy_Version_Idx);
    }else{
        Msg_N_Bits          = 32;  // fixed by Chia-Hao Chang
        ul_msg.msg.N_bits   = 32;
        this->flip_ul_ndi();
        set_ul_msg(Msg_N_Bits);
    }
}
void LTE_fdd_enb_user::set_retransmission()
{
    this->update_rv_idx();
    if(this->alloc_chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
    {
        set_dl_msg(Msg_N_Bits, Msg_Bits, Redundancy_Version_Idx);
    }else{
        set_ul_msg(Msg_N_Bits);                    
    }   
}

void LTE_fdd_enb_user::update_rv_idx(void)
{
    if(Redundancy_Version_Idx==0)
    {
        Redundancy_Version_Idx = 2;
    }else if(Redundancy_Version_Idx==2){
        Redundancy_Version_Idx = 1;
    }else if(Redundancy_Version_Idx==1){
        Redundancy_Version_Idx = 3;
    }else{
        Redundancy_Version_Idx = 0;
    }
}

void LTE_fdd_enb_user::get_information_bits(void)
{
    // Get the information bits
    uint8* tmp = Msg_Bits;
    value_2_bits(tx_information, &tmp, 8);
    value_2_bits(tx_information, &tmp, 8);

    if(tx_information>='A' && tx_information<'Z')
    {
        tx_information += 1;
    }else{
        tx_information = 'A';
    }
}