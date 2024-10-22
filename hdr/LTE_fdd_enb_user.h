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

    File: LTE_fdd_enb_user.h

    Description: Contains all the definitions for the LTE FDD eNodeB
                 user class.

    Revision History
    ----------    -------------    --------------------------------------------
    11/09/2013    Ben Wojtowicz    Created file
    05/04/2014    Ben Wojtowicz    Added radio bearer support.
    06/15/2014    Ben Wojtowicz    Added initialize routine.
    08/03/2014    Ben Wojtowicz    Refactored user identities.
    09/03/2014    Ben Wojtowicz    Added ciphering and integrity algorithm
                                   storing.

*******************************************************************************/

#ifndef __LTE_FDD_ENB_USER_H__
#define __LTE_FDD_ENB_USER_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "LTE_fdd_enb_interface.h"
#include "LTE_fdd_enb_cnfg_db.h"
#include "LTE_fdd_enb_rb.h"
#include "liblte_mac.h"
#include "liblte_phy.h"
#include "typedefs.h"
#include <string>
#include <vector>

/*******************************************************************************
                              DEFINES
*******************************************************************************/


/*******************************************************************************
                              FORWARD DECLARATIONS
*******************************************************************************/
using namespace std;

/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/

typedef struct{
    uint64 imsi;
    uint64 imei;
}LTE_FDD_ENB_USER_ID_STRUCT;

typedef struct{
    uint32 nas_count_ul;
    uint32 nas_count_dl;
    uint8  rand[16];
    uint8  res[8];
    uint8  ck[16];
    uint8  ik[16];
    uint8  autn[16];
    uint8  k_nas_enc[32];
    uint8  k_nas_int[32];
}LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT;

/*******************************************************************************
                              CLASS DECLARATIONS
*******************************************************************************/

class LTE_fdd_enb_user
{
public:
    // Constructor/Destructor
    LTE_fdd_enb_user(uint16 _c_rnti);
    ~LTE_fdd_enb_user();

    // Initialize
    void init(void);

    // Identity
    void set_id(LTE_FDD_ENB_USER_ID_STRUCT *identity);
    LTE_FDD_ENB_USER_ID_STRUCT* get_id(void);
    bool is_id_set(void);
    void set_temp_id(uint64 id);
    uint64 get_temp_id(void);
    std::string get_imsi_str(void);
    uint64 get_imsi_num(void);
    std::string get_imei_str(void);
    uint64 get_imei_num(void);
    void set_c_rnti(uint16 _c_rnti);
    uint16 get_c_rnti(void);
    bool is_c_rnti_set(void);
    void set_auth_vec(LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT *av);
    LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT* get_auth_vec(void);
    void increment_nas_count_dl(void);
    void increment_nas_count_ul(void);
    bool is_auth_vec_set(void);

    // Capabilities
    void set_eea_support(uint8 eea, bool support);
    bool get_eea_support(uint8 eea);
    void set_eia_support(uint8 eia, bool support);
    bool get_eia_support(uint8 eia);
    void set_uea_support(uint8 uea, bool support);
    bool get_uea_support(uint8 uea);
    bool is_uea_set(void);
    void set_uia_support(uint8 uia, bool support);
    bool get_uia_support(uint8 uia);
    bool is_uia_set(void);
    void set_gea_support(uint8 gea, bool support);
    bool get_gea_support(uint8 gea);
    bool is_gea_set(void);

    // Radio Bearers
    /*
    void get_srb0(LTE_fdd_enb_rb **rb);
    LTE_FDD_ENB_ERROR_ENUM setup_srb1(LTE_fdd_enb_rb **rb);
    LTE_FDD_ENB_ERROR_ENUM teardown_srb1(void);
    LTE_FDD_ENB_ERROR_ENUM get_srb1(LTE_fdd_enb_rb **rb);
    LTE_FDD_ENB_ERROR_ENUM setup_srb2(LTE_fdd_enb_rb **rb);
    LTE_FDD_ENB_ERROR_ENUM teardown_srb2(void);
    LTE_FDD_ENB_ERROR_ENUM  get_srb2(LTE_fdd_enb_rb **rb);
    */
    // MAC
    bool get_dl_ndi(void);
    void flip_dl_ndi(void);
    bool get_ul_ndi(void);
    void flip_ul_ndi(void);
    LIBLTE_MAC_PDU_STRUCT pusch_mac_pdu;

    // Generic
    void set_delete_at_idle(bool dai);
    bool get_delete_at_idle(void);

    //
    LTE_FDD_ENB_SYS_INFO_STRUCT        sys_info;

    // User's message

    LIBLTE_PHY_ALLOCATION_STRUCT        ul_msg;
    LIBLTE_PHY_ALLOCATION_STRUCT        dl_msg;
    LIBLTE_PHY_CHAN_TYPE_ENUM           alloc_chan_type;
    vector<LIBLTE_BIT_MSG_STRUCT>  uplink_received_msg;

    void set_new_transmission();
    void set_new_transmission(string& msg_char, int32 num_bits, uint8 mcs);
    void set_retransmission();

    void set_ul_msg(uint32 N_bits);
    void set_dl_msg(uint32 N_bits, 
                    uint8* msg_bits, 
                    uint32 rv_idx);
    
    void set_allocation(LIBLTE_BIT_MSG_STRUCT*          msg_bits,
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
                        bool                            ndi);

    // For Demo...
    uint8 Modulation_and_CodingScheme;

private:
    // Identity
    LTE_FDD_ENB_USER_ID_STRUCT               id;
    LTE_FDD_ENB_AUTHENTICATION_VECTOR_STRUCT auth_vec;
    uint64                                   temp_id;
    uint32                                   c_rnti;
    bool                                     id_set;
    bool                                     c_rnti_set;
    bool                                     auth_vec_set;

    // Capabilities
    bool eea_support[8];
    bool eia_support[8];
    bool uea_support[8];
    bool uea_set;
    bool uia_support[8];
    bool uia_set;
    bool gea_support[8];
    bool gea_set;

    // Radio Bearerss
    //LTE_fdd_enb_rb *srb0;
    //LTE_fdd_enb_rb *srb1;
    //LTE_fdd_enb_rb *srb2;
    //LTE_fdd_enb_rb *drb[8];

    // MAC
    bool    dl_ndi;
    bool    ul_ndi;

    // Generic
    bool    delete_at_idle;

    // Message
    void        get_information_bits(void);
    void        update_rv_idx(void);
    uint32      Msg_Alloc_PRB[LIBLTE_PHY_N_SLOTS_PER_SUBFR][6];
    uint32      Msg_N_Bits;
    uint32      Redundancy_Version_Idx;
    uint8       Msg_Bits[768];
    uint8       tx_information;

};

#endif /* __LTE_FDD_ENB_USER_H__ */
