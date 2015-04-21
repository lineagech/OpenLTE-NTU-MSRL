/********************************************/
/*********** Include ************************/
/********************************************/
#include "hdr/LTE_fdd_main.h"

/*******************************************/
/*********** Forward Declaration ***********/
/*******************************************/
class LTE_fdd_enb_phy;

/********************************************/
/*********** Global Variables ***************/
/********************************************/

boost::mutex phy_instance_mutex;
LTE_File lte_file_OFDM_symbol("OFDM_symbol.dat");

LTE_fdd_enb_phy* LTE_fdd_enb_phy::instance = NULL;

/********************************************/
/*********** Functions **********************/
/********************************************/
LTE_fdd_enb_phy* LTE_fdd_enb_phy::get_instance(void)
{
    boost::mutex::scoped_lock lock(phy_instance_mutex);

    if(NULL == instance)
    {
        instance = new LTE_fdd_enb_phy();
    }

    return(instance);
}

void LTE_fdd_enb_phy::LTE_fdd_enb_cnfg_db()
{
    // Parameter initialization
    var_map_double[LTE_FDD_ENB_PARAM_BANDWIDTH]                = 1.4;
    var_map_int64[LTE_FDD_ENB_PARAM_FREQ_BAND]                 = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_DL_EARFCN]                 = liblte_interface_first_dl_earfcn[0];
    var_map_int64[LTE_FDD_ENB_PARAM_UL_EARFCN]                 = liblte_interface_get_corresponding_ul_earfcn(liblte_interface_first_dl_earfcn[0]);
    var_map_int64[LTE_FDD_ENB_PARAM_DL_CENTER_FREQ]            = liblte_interface_dl_earfcn_to_frequency(liblte_interface_first_dl_earfcn[0]);
    var_map_int64[LTE_FDD_ENB_PARAM_UL_CENTER_FREQ]            = liblte_interface_ul_earfcn_to_frequency(liblte_interface_get_corresponding_ul_earfcn(liblte_interface_first_dl_earfcn[0]));
    var_map_int64[LTE_FDD_ENB_PARAM_N_RB_DL]                   = /*LIBLTE_PHY_N_RB_DL_20MHZ/**/LIBLTE_PHY_N_RB_DL_1_4MHZ;
    var_map_int64[LTE_FDD_ENB_PARAM_N_RB_UL]                   = LIBLTE_PHY_N_RB_UL_1_4MHZ;
    var_map_int64[LTE_FDD_ENB_PARAM_DL_BW]                     = /*LIBLTE_RRC_DL_BANDWIDTH_75/**/LIBLTE_RRC_DL_BANDWIDTH_6;
    var_map_int64[LTE_FDD_ENB_PARAM_N_SC_RB_DL]                = LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP;
    var_map_int64[LTE_FDD_ENB_PARAM_N_SC_RB_UL]                = LIBLTE_PHY_N_SC_RB_UL;
    var_map_int64[LTE_FDD_ENB_PARAM_N_ANT]                     = 1;
    var_map_int64[LTE_FDD_ENB_PARAM_N_ID_CELL]                 = 299;
    var_map_int64[LTE_FDD_ENB_PARAM_N_ID_2]                    = 2;
    var_map_int64[LTE_FDD_ENB_PARAM_N_ID_1]                    = 99;
    var_map_uint32[LTE_FDD_ENB_PARAM_MCC]                      = 0xFFFFF001;
    var_map_uint32[LTE_FDD_ENB_PARAM_MNC]                      = 0xFFFFFF01;
    var_map_int64[LTE_FDD_ENB_PARAM_CELL_ID]                   = 1;
    var_map_int64[LTE_FDD_ENB_PARAM_TRACKING_AREA_CODE]        = 1;
    var_map_int64[LTE_FDD_ENB_PARAM_Q_RX_LEV_MIN]              = -140;
    var_map_int64[LTE_FDD_ENB_PARAM_P0_NOMINAL_PUSCH]          = -70;
    var_map_int64[LTE_FDD_ENB_PARAM_P0_NOMINAL_PUCCH]          = -96;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB3_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_Q_HYST]                    = LIBLTE_RRC_Q_HYST_DB_0;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB4_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB5_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB6_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB7_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_SIB8_PRESENT]              = 0;
    var_map_int64[LTE_FDD_ENB_PARAM_SEARCH_WIN_SIZE]           = 0;
    var_map_uint32[LTE_FDD_ENB_PARAM_SYSTEM_INFO_VALUE_TAG]    = 1;
    var_map_int64[LTE_FDD_ENB_PARAM_SYSTEM_INFO_WINDOW_LENGTH] = LIBLTE_RRC_SI_WINDOW_LENGTH_MS1;
    var_map_int64[LTE_FDD_ENB_PARAM_PHICH_RESOURCE]            = LIBLTE_RRC_PHICH_RESOURCE_1;
    var_map_int64[LTE_FDD_ENB_PARAM_N_SCHED_INFO]              = 1;
    var_map_int64[LTE_FDD_ENB_PARAM_SYSTEM_INFO_PERIODICITY]   = LIBLTE_RRC_SI_PERIODICITY_RF8;
    var_map_uint32[LTE_FDD_ENB_PARAM_DEBUG_TYPE]               = 0xFFFFFFFF;
    var_map_uint32[LTE_FDD_ENB_PARAM_DEBUG_LEVEL]              = 0xFFFFFFFF;
    var_map_int64[LTE_FDD_ENB_PARAM_ENABLE_PCAP]               = 0;

    std::cerr<<"---BS Configuration---"<<std::endl;
    std::cerr<<setw(30)<<"Cell ID : "<<var_map_int64[LTE_FDD_ENB_PARAM_N_ID_CELL] <<std::endl;
    std::cerr<<setw(30)<<"Antenna number : "<<var_map_int64[LTE_FDD_ENB_PARAM_N_ANT]<<std::endl;  
    std::cerr<<setw(30)<<"Bandwidth : "<<var_map_double[LTE_FDD_ENB_PARAM_BANDWIDTH]<<std::endl;
    std::cerr<<setw(30)<<"Downlink RB number : "<<var_map_int64[LTE_FDD_ENB_PARAM_N_RB_DL]<<std::endl;
    std::cerr<<setw(30)<<"Subcarrier number per RB : "<<var_map_int64[LTE_FDD_ENB_PARAM_N_SC_RB_DL]<<std::endl;
    std::cerr<<"----------------------"<<std::endl;
}

void LTE_fdd_enb_phy::construct_sys_info()
{
    /*build system information*/
    LTE_fdd_enb_cnfg_db();

    std::map<LTE_FDD_ENB_PARAM_ENUM, double>::iterator  double_iter;
    std::map<LTE_FDD_ENB_PARAM_ENUM, int64>::iterator   int64_iter;
    std::map<LTE_FDD_ENB_PARAM_ENUM, uint32>::iterator  uint32_iter;
    LIBLTE_RRC_SIB_TYPE_ENUM                            sib_array[6];
    LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT                    bcch_dlsch_msg;
    uint32                                              num_sibs      = 0;
    uint32                                              sib_idx       = 0;
    uint32                                              N_sibs_to_map = 0;
    uint32                                              i;
    uint32                                              j;
    
    // MIB
    double_iter = var_map_double.find(LTE_FDD_ENB_PARAM_BANDWIDTH);
    if(var_map_double.end() != double_iter)
    {
        for(i=0; i<LIBLTE_RRC_DL_BANDWIDTH_N_ITEMS; i++)
        {
            if((*double_iter).second == liblte_rrc_dl_bandwidth_num[i])
            {
                sys_info.mib.dl_bw = (LIBLTE_RRC_DL_BANDWIDTH_ENUM)i;
                break;
            }
        }
    }
    sys_info.mib.phich_config.dur = LIBLTE_RRC_PHICH_DURATION_NORMAL;
    sys_info.mib.phich_config.res = LIBLTE_RRC_PHICH_RESOURCE_1;
    
    // Determine which SIBs need to be mapped
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB3_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_3;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB4_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_4;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB5_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_5;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB6_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_6;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB7_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_7;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB8_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_8;
    }
    
    // Initialize the scheduling info
    sys_info.sib1.N_sched_info                     = 1;
    sys_info.sib1.sched_info[0].N_sib_mapping_info = 0;
    
    // Map the SIBs
    while(num_sibs > 0)
    {
        // Determine how many SIBs can be mapped to this scheduling info
        if(1 == sys_info.sib1.N_sched_info)
        {
            if(0                         == sys_info.sib1.sched_info[0].N_sib_mapping_info &&
               LIBLTE_RRC_DL_BANDWIDTH_6 != sys_info.mib.dl_bw)
            {
                N_sibs_to_map = 1;
            }else{
                N_sibs_to_map                                                           = 2;
                sys_info.sib1.sched_info[sys_info.sib1.N_sched_info].N_sib_mapping_info = 0;
                sys_info.sib1.sched_info[sys_info.sib1.N_sched_info].si_periodicity     = LIBLTE_RRC_SI_PERIODICITY_RF8;
                sys_info.sib1.N_sched_info++;
            }
        }else{
            if(2 > sys_info.sib1.sched_info[sys_info.sib1.N_sched_info-1].N_sib_mapping_info)
            {
                N_sibs_to_map = 2 - sys_info.sib1.sched_info[sys_info.sib1.N_sched_info-1].N_sib_mapping_info;
            }else{
                N_sibs_to_map                                                           = 2;
                sys_info.sib1.sched_info[sys_info.sib1.N_sched_info].N_sib_mapping_info = 0;
                sys_info.sib1.sched_info[sys_info.sib1.N_sched_info].si_periodicity     = LIBLTE_RRC_SI_PERIODICITY_RF8;
                sys_info.sib1.N_sched_info++;
            }
        }
    
        // Map the SIBs for this scheduling info
        for(i=0; i<N_sibs_to_map; i++)
        {
            sys_info.sib1.sched_info[sys_info.sib1.N_sched_info-1].sib_mapping_info[sys_info.sib1.sched_info[sys_info.sib1.N_sched_info-1].N_sib_mapping_info].sib_type = sib_array[sib_idx++];
            sys_info.sib1.sched_info[sys_info.sib1.N_sched_info-1].N_sib_mapping_info++;
            num_sibs--;

            if(0 == num_sibs)
            {
                break;
            }
        }
    }

    // SIB1
    sys_info.sib1.N_plmn_ids = 1;
    uint32_iter = var_map_uint32.find(LTE_FDD_ENB_PARAM_MCC);
    if(var_map_uint32.end() != uint32_iter)
    {
        sys_info.sib1.plmn_id[0].id.mcc = ((*uint32_iter).second) & 0xFFFF;
        sys_info.mcc                    = 0;
        for(i=0; i<3; i++)
        {
            sys_info.mcc *= 10;
            sys_info.mcc |= (((*uint32_iter).second) >> (2-i)*4) & 0xF;
        }
    }
    uint32_iter = var_map_uint32.find(LTE_FDD_ENB_PARAM_MNC);
    if(var_map_uint32.end() != uint32_iter)
    {
        sys_info.sib1.plmn_id[0].id.mnc = ((*uint32_iter).second) & 0xFFFF;
        sys_info.mnc                    = 0;
        if(((((*uint32_iter).second) >> 8) & 0xF) == 0xF)
        {
            for(i=0; i<2; i++)
            {
                sys_info.mnc *= 10;
                sys_info.mnc |= (((*uint32_iter).second) >> (1-i)*4) & 0xF;
            }
        }else{
            for(i=0; i<3; i++)
            {
                sys_info.mnc *= 10;
                sys_info.mnc |= (((*uint32_iter).second) >> (2-i)*4) & 0xF;
            }
        }
    }
    sys_info.sib1.plmn_id[0].resv_for_oper         = LIBLTE_RRC_NOT_RESV_FOR_OPER;
    sys_info.sib1.cell_barred                      = LIBLTE_RRC_CELL_NOT_BARRED;
    sys_info.sib1.intra_freq_reselection           = LIBLTE_RRC_INTRA_FREQ_RESELECTION_ALLOWED;
    sys_info.sib1.si_window_length                 = LIBLTE_RRC_SI_WINDOW_LENGTH_MS2;
    sys_info.sib1.sf_assignment                    = LIBLTE_RRC_SUBFRAME_ASSIGNMENT_0;
    sys_info.sib1.special_sf_patterns              = LIBLTE_RRC_SPECIAL_SUBFRAME_PATTERNS_0;
    int64_iter                                     = var_map_int64.find(LTE_FDD_ENB_PARAM_CELL_ID);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib1.cell_id = (*int64_iter).second;
    }
    sys_info.sib1.csg_id = 0;
    int64_iter           = var_map_int64.find(LTE_FDD_ENB_PARAM_TRACKING_AREA_CODE);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib1.tracking_area_code = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_Q_RX_LEV_MIN);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib1.q_rx_lev_min = (*int64_iter).second;
    }
    sys_info.sib1.csg_indication      = 0;
    sys_info.sib1.q_rx_lev_min_offset = 1;
    int64_iter                        = var_map_int64.find(LTE_FDD_ENB_PARAM_FREQ_BAND);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib1.freq_band_indicator = liblte_interface_band_num[(*int64_iter).second];
    }
    uint32_iter = var_map_uint32.find(LTE_FDD_ENB_PARAM_SYSTEM_INFO_VALUE_TAG);
    if(var_map_uint32.end() != uint32_iter)
    {
        sys_info.sib1.system_info_value_tag = (*uint32_iter).second++;
    }
    sys_info.sib1.p_max_present = true;
    sys_info.sib1.p_max         = 23;
    sys_info.sib1.tdd           = false;
    
    // SIB2
    sys_info.sib2.ac_barring_info_present                                                      = false;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.num_ra_preambles                              = LIBLTE_RRC_NUMBER_OF_RA_PREAMBLES_N4;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present                = false;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.pwr_ramping_step                              = LIBLTE_RRC_POWER_RAMPING_STEP_DB6;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr                   = LIBLTE_RRC_PREAMBLE_INITIAL_RECEIVED_TARGET_POWER_DBM_N90;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.preamble_trans_max                            = LIBLTE_RRC_PREAMBLE_TRANS_MAX_N200;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.ra_resp_win_size                              = LIBLTE_RRC_RA_RESPONSE_WINDOW_SIZE_SF7;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.mac_con_res_timer                             = LIBLTE_RRC_MAC_CONTENTION_RESOLUTION_TIMER_SF64;
    sys_info.sib2.rr_config_common_sib.rach_cnfg.max_harq_msg3_tx                              = 1;
    sys_info.sib2.rr_config_common_sib.bcch_cnfg.modification_period_coeff                     = LIBLTE_RRC_MODIFICATION_PERIOD_COEFF_N2;
    sys_info.sib2.rr_config_common_sib.pcch_cnfg.default_paging_cycle                          = LIBLTE_RRC_DEFAULT_PAGING_CYCLE_RF256;
    sys_info.sib2.rr_config_common_sib.pcch_cnfg.nB                                            = LIBLTE_RRC_NB_ONE_T;
    sys_info.sib2.rr_config_common_sib.prach_cnfg.root_sequence_index                          = 0;
    sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index           = 6; // fixed by Chia-Hao Chang
    sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag              = false;
    sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config = 1;
    sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset            = 0;
    sys_info.sib2.rr_config_common_sib.pdsch_cnfg.rs_power                                     = 0;
    sys_info.sib2.rr_config_common_sib.pdsch_cnfg.p_b                                          = 0;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.n_sb                                         = 1;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.hopping_mode                                 = LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.pusch_hopping_offset                         = 0;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.enable_64_qam                                = true;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled                  = false;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch                 = 0;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled               = false;
    sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift                           = 0;
    sys_info.sib2.rr_config_common_sib.pucch_cnfg.delta_pucch_shift                            = LIBLTE_RRC_DELTA_PUCCH_SHIFT_DS1;
    sys_info.sib2.rr_config_common_sib.pucch_cnfg.n_rb_cqi                                     = 0;
    sys_info.sib2.rr_config_common_sib.pucch_cnfg.n_cs_an                                      = 0;
    sys_info.sib2.rr_config_common_sib.pucch_cnfg.n1_pucch_an                                  = 0;
    sys_info.sib2.rr_config_common_sib.srs_ul_cnfg.present                                     = false;
    int64_iter                                                                                 = var_map_int64.find(LTE_FDD_ENB_PARAM_P0_NOMINAL_PUSCH);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch = (*int64_iter).second;
    }
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.alpha = LIBLTE_RRC_UL_POWER_CONTROL_ALPHA_1;
    int64_iter                                           = var_map_int64.find(LTE_FDD_ENB_PARAM_P0_NOMINAL_PUCCH);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch = (*int64_iter).second;
    }
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1_0;
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1B_1;
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2_0;
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2A_0;
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2B_0;
    sys_info.sib2.rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3         = -2;
    sys_info.sib2.rr_config_common_sib.ul_cp_length                            = LIBLTE_RRC_UL_CP_LENGTH_1;
    sys_info.sib2.ue_timers_and_constants.t300                                 = LIBLTE_RRC_T300_MS1000;
    sys_info.sib2.ue_timers_and_constants.t301                                 = LIBLTE_RRC_T301_MS1000;
    sys_info.sib2.ue_timers_and_constants.t310                                 = LIBLTE_RRC_T310_MS1000;
    sys_info.sib2.ue_timers_and_constants.n310                                 = LIBLTE_RRC_N310_N20;
    sys_info.sib2.ue_timers_and_constants.t311                                 = LIBLTE_RRC_T311_MS1000;
    sys_info.sib2.ue_timers_and_constants.n311                                 = LIBLTE_RRC_N311_N10;
    sys_info.sib2.arfcn_value_eutra.present                                    = false;
    sys_info.sib2.ul_bw.present                                                = false;
    sys_info.sib2.additional_spectrum_emission                                 = 1;
    sys_info.sib2.mbsfn_subfr_cnfg_list_size                                   = 0;
    sys_info.sib2.time_alignment_timer                                         = LIBLTE_RRC_TIME_ALIGNMENT_TIMER_SF500;
    
    // SIB3
    sys_info.sib3_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB3_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib3_present = true;
        int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_Q_HYST);
        if(var_map_int64.end() != int64_iter)
        {
            sys_info.sib3.q_hyst = (LIBLTE_RRC_Q_HYST_ENUM)(*int64_iter).second;
        }
        sys_info.sib3.speed_state_resel_params.present = false;
        sys_info.sib3.s_non_intra_search_present       = false;
        sys_info.sib3.thresh_serving_low               = 0;
        sys_info.sib3.cell_resel_prio                  = 0;
        sys_info.sib3.q_rx_lev_min                     = sys_info.sib1.q_rx_lev_min;
        sys_info.sib3.p_max_present                    = true;
        sys_info.sib3.p_max                            = sys_info.sib1.p_max;
        sys_info.sib3.s_intra_search_present           = false;
        sys_info.sib3.allowed_meas_bw_present          = false;
        sys_info.sib3.presence_ant_port_1              = false;
        sys_info.sib3.neigh_cell_cnfg                  = 0;
        sys_info.sib3.t_resel_eutra                    = 0;
        sys_info.sib3.t_resel_eutra_sf_present         = false;
    }
    
    // SIB4
    sys_info.sib4_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB4_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib4_present                         = true;
        sys_info.sib4.intra_freq_neigh_cell_list_size = 0;
        sys_info.sib4.intra_freq_black_cell_list_size = 0;
        sys_info.sib4.csg_phys_cell_id_range_present  = false;
    }
    
    // SIB5
    sys_info.sib5_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB5_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib5_present                           = true;
        sys_info.sib5.inter_freq_carrier_freq_list_size = 0;
    }
    
    // SIB6
    sys_info.sib6_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB6_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib6_present                         = true;
        sys_info.sib6.carrier_freq_list_utra_fdd_size = 0;
        sys_info.sib6.carrier_freq_list_utra_tdd_size = 0;
        sys_info.sib6.t_resel_utra                    = 1;
        sys_info.sib6.t_resel_utra_sf_present         = false;
    }
    
    // SIB7
    sys_info.sib7_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB7_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib7_present                      = true;
        sys_info.sib7.t_resel_geran                = 1;
        sys_info.sib7.t_resel_geran_sf_present     = false;
        sys_info.sib7.carrier_freqs_info_list_size = 0;
    }
    
    // SIB8
    sys_info.sib8_present = false;
    int64_iter            = var_map_int64.find(LTE_FDD_ENB_PARAM_SIB8_PRESENT);
    if(var_map_int64.end() != int64_iter &&
       1                   == (*int64_iter).second)
    {
        sys_info.sib8_present                 = true;
        sys_info.sib8.sys_time_info_present   = false;
        sys_info.sib8.search_win_size_present = true;
        int64_iter                            = var_map_int64.find(LTE_FDD_ENB_PARAM_SEARCH_WIN_SIZE);
        if(var_map_int64.end() != int64_iter)
        {
            sys_info.sib8.search_win_size = (*int64_iter).second;
        }
        sys_info.sib8.params_hrpd_present  = false;
        sys_info.sib8.params_1xrtt_present = false;
    }
    
    // Pack SIB1
    bcch_dlsch_msg.N_sibs           = 0;
    bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1;
    memcpy(&bcch_dlsch_msg.sibs[0].sib, &sys_info.sib1, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT));
    liblte_rrc_pack_bcch_dlsch_msg(&bcch_dlsch_msg,
                                   &sys_info.sib1_alloc.msg);
    sys_info.sib1_alloc.pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
    sys_info.sib1_alloc.mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    sys_info.sib1_alloc.rv_idx         = 0; // 36.321 section 5.3.1
    sys_info.sib1_alloc.N_codewords    = 1;
    sys_info.sib1_alloc.rnti           = LIBLTE_MAC_SI_RNTI;
    sys_info.sib1_alloc.tx_mode        = 1;
    
    // Pack additional SIBs
    bcch_dlsch_msg.N_sibs           = 1;
    bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2;
    memcpy(&bcch_dlsch_msg.sibs[0].sib, &sys_info.sib2, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT));
    if(0 != sys_info.sib1.sched_info[0].N_sib_mapping_info)
    {
        switch(sys_info.sib1.sched_info[0].sib_mapping_info[0].sib_type)
        {
        case LIBLTE_RRC_SIB_TYPE_3:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib3, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT));
            break;
        case LIBLTE_RRC_SIB_TYPE_4:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib4, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT));
            break;
        case LIBLTE_RRC_SIB_TYPE_5:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib5, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT));
            break;
        case LIBLTE_RRC_SIB_TYPE_6:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib6, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT));
            break;
        case LIBLTE_RRC_SIB_TYPE_7:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib7, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT));
            break;
        case LIBLTE_RRC_SIB_TYPE_8:
            bcch_dlsch_msg.N_sibs++;
            bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8;
            memcpy(&bcch_dlsch_msg.sibs[1].sib, &sys_info.sib8, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT));
            break;
        default:
            break;
        }
    }
    liblte_rrc_pack_bcch_dlsch_msg(&bcch_dlsch_msg,
                                   &sys_info.sib_alloc[0].msg);
    sys_info.sib_alloc[0].pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
    sys_info.sib_alloc[0].mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    sys_info.sib_alloc[0].rv_idx         = 0; // 36.321 section 5.3.1
    sys_info.sib_alloc[0].N_codewords    = 1;
    sys_info.sib_alloc[0].rnti           = LIBLTE_MAC_SI_RNTI;
    sys_info.sib_alloc[0].tx_mode        = 1;
    for(i=1; i<sys_info.sib1.N_sched_info; i++)
    {
        bcch_dlsch_msg.N_sibs = sys_info.sib1.sched_info[i].N_sib_mapping_info;
        for(j=0; j<bcch_dlsch_msg.N_sibs; j++)
        {
            switch(sys_info.sib1.sched_info[i].sib_mapping_info[j].sib_type)
            {
            case LIBLTE_RRC_SIB_TYPE_3:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib3, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT));
                break;
            case LIBLTE_RRC_SIB_TYPE_4:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib4, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT));
                break;
            case LIBLTE_RRC_SIB_TYPE_5:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib5, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT));
                break;
            case LIBLTE_RRC_SIB_TYPE_6:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib6, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT));
                break;
            case LIBLTE_RRC_SIB_TYPE_7:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib7, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT));
                break;
            case LIBLTE_RRC_SIB_TYPE_8:
                bcch_dlsch_msg.sibs[j].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8;
                memcpy(&bcch_dlsch_msg.sibs[j].sib, &sys_info.sib8, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT));
                break;
            default:
                break;
            }
        }
        liblte_rrc_pack_bcch_dlsch_msg(&bcch_dlsch_msg,
                                       &sys_info.sib_alloc[i].msg);
        sys_info.sib_alloc[i].pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
        sys_info.sib_alloc[i].mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        sys_info.sib_alloc[i].rv_idx         = 0; // 36.321 section 5.3.1
        sys_info.sib_alloc[i].N_codewords    = 1;
        sys_info.sib_alloc[i].rnti           = LIBLTE_MAC_SI_RNTI;
        sys_info.sib_alloc[i].tx_mode        = 1;
    }

    // Generic parameters
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_ANT);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_ant = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_ID_CELL);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_id_cell = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_ID_1);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_id_1 = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_ID_2);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_id_2 = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_RB_DL);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_rb_dl = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_RB_UL);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_rb_ul = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_SC_RB_DL);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_sc_rb_dl = (*int64_iter).second;
    }
    int64_iter = var_map_int64.find(LTE_FDD_ENB_PARAM_N_SC_RB_UL);
    if(var_map_int64.end() != int64_iter)
    {
        sys_info.N_sc_rb_ul = (*int64_iter).second;
    }
    sys_info.si_periodicity_T = liblte_rrc_si_periodicity_num[sys_info.sib1.sched_info[0].si_periodicity];
    sys_info.si_win_len       = liblte_rrc_si_window_length_num[sys_info.sib1.si_window_length];

    // PCAP variables
    sys_info.mib_pcap_sent       = false;
    sys_info.sib1_pcap_sent      = false;
    sys_info.sib_pcap_sent[0]    = false;
    sys_info.sib_pcap_sent[1]    = false;
    sys_info.sib_pcap_sent[2]    = false;
    sys_info.sib_pcap_sent[3]    = false;
    sys_info.continuous_sib_pcap = false;
}

void LTE_fdd_enb_phy::start(LTE_fdd_enb_interface *iface)
{
    LIBLTE_PHY_FS_ENUM   fs;
    uint32               i;
    uint32               j;
    uint32               k;
    uint32               samp_rate;
    uint8                prach_cnfg_idx;

    /*build thread to receive the msg from MAC layer*/
    //LTE_fdd_enb_msgq_cb  cb(&LTE_fdd_enb_msgq_cb_wrapper<LTE_fdd_enb_phy, &LTE_fdd_enb_phy::handle_mac_msg>, this);

    /************************************************/

    if(!started)
    {
        // Get the latest sys info
        //update_sys_info();
        construct_sys_info();
    
        // Initialize phy
        samp_rate = get_sample_rate();
        if(30720000 == samp_rate)
        {
            fs = LIBLTE_PHY_FS_30_72MHZ;
        }else if(15360000 == samp_rate){
            fs = LIBLTE_PHY_FS_15_36MHZ;
        }else if(7680000 == samp_rate){
            fs = LIBLTE_PHY_FS_7_68MHZ;
        }else if(3840000 == samp_rate){
            fs = LIBLTE_PHY_FS_3_84MHZ;
        }else if(1920000 == samp_rate){
            fs = LIBLTE_PHY_FS_1_92MHZ;
        }else{
            iface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_ERROR,
                                  LTE_FDD_ENB_DEBUG_LEVEL_PHY,
                                  __FILE__,
                                  __LINE__,
                                  "Invalid sample rate %u",
                                  samp_rate);
        }
        liblte_phy_init(&phy_struct,
                        fs,
                        sys_info.N_id_cell,
                        sys_info.N_ant,
                        sys_info.N_rb_dl,
                        sys_info.N_sc_rb_dl,
                        liblte_rrc_phich_resource_num[sys_info.mib.phich_config.res]);
        liblte_phy_ul_init(phy_struct,
                           sys_info.N_id_cell,
                           sys_info.sib2.rr_config_common_sib.prach_cnfg.root_sequence_index,
                           sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index>>4,
                           sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config,
                           sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag,
                           sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch,
                           sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled,
                           sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled,
                           sys_info.sib2.rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift,
                           0);
    
        // Downlink
        for(i=0; i<10; i++)
        {
            dl_schedule[i].current_tti            = i;
            dl_schedule[i].dl_allocations.N_alloc = 0;
            dl_schedule[i].ul_allocations.N_alloc = 0;
            ul_schedule[i].current_tti            = i;
            ul_schedule[i].decodes.N_alloc        = 0;
        }
        pcfich.cfi = 2; // FIXME: Make this dynamic every subfr
        for(i=0; i<10; i++)
        {
            for(j=0; j<25; j++)
            {
                for(k=0; k<8; k++)
                {
                    phich[i].present[j][k] = false;
                }
            }
        }
        pdcch.N_alloc        = 0;
        pdcch.N_symbs        = 2; // FIXME: Make this dynamic every subfr
        dl_subframe.num      = 0;
        dl_current_tti       = 0;
        last_rts_current_tti = 0;
        late_subfr           = false;
    
        // Uplink
        ul_current_tti = (LTE_FDD_ENB_CURRENT_TTI_MAX + 1) - 2;
        prach_cnfg_idx = sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index;
        if(prach_cnfg_idx ==  0 ||
           prach_cnfg_idx ==  1 ||
           prach_cnfg_idx ==  2 ||
           prach_cnfg_idx == 15 ||
           prach_cnfg_idx == 16 ||
           prach_cnfg_idx == 17 ||
           prach_cnfg_idx == 18 ||
           prach_cnfg_idx == 31 ||
           prach_cnfg_idx == 32 ||
           prach_cnfg_idx == 33 ||
           prach_cnfg_idx == 34 ||
           prach_cnfg_idx == 47 ||
           prach_cnfg_idx == 48 ||
           prach_cnfg_idx == 49 ||
           prach_cnfg_idx == 50 ||
           prach_cnfg_idx == 63)
        {
            prach_sfn_mod = 2;
        }else{
            prach_sfn_mod = 1;
        }
        prach_subfn_zero_allowed = true;
        switch(prach_cnfg_idx % 16)
        {
        case 0:
            // Intentional fall through
        case 3:
            prach_subfn_mod   = 10;
            prach_subfn_check = 1;
            break;
        case 1:
            // Intentional fall through
        case 4:
            prach_subfn_mod   = 10;
            prach_subfn_check = 4;
            break;
        case 2:
            // Intentional fall through
        case 5:
            prach_subfn_mod   = 10;
            prach_subfn_check = 7;
            break;
        case 6:
            prach_subfn_mod   = 5;
            prach_subfn_check = 1;
            break;
        case 7:
            prach_subfn_mod   = 5;
            prach_subfn_check = 2;
            break;
        case 8:
            prach_subfn_mod   = 5;
            prach_subfn_check = 3;
            break;
        case 9:
            prach_subfn_mod   = 3;
            prach_subfn_check = 1;
            break;
        case 10:
            prach_subfn_mod   = 3;
            prach_subfn_check = 2;
            break;
        case 11:
            prach_subfn_mod          = 3;
            prach_subfn_check        = 0;
            prach_subfn_zero_allowed = false;
            break;
        case 12:
            prach_subfn_mod   = 2;
            prach_subfn_check = 0;
            break;
        case 13:
            prach_subfn_mod   = 2;
            prach_subfn_check = 1;
            break;
        case 14:
            prach_subfn_mod   = 1;
            prach_subfn_check = 0;
            break;
        case 15:
            prach_subfn_mod   = 10;
            prach_subfn_check = 9;
            break;
        }
    
        // Communication
        //mac_comm_msgq = new LTE_fdd_enb_msgq("mac_phy_mq",
                                                 //cb);
        //phy_mac_mq    = new boost::interprocess::message_queue(boost::interprocess::open_only,
        //                                                           "phy_mac_mq");
        
        interface = iface;
        started   = true;
    }
}

void LTE_fdd_enb_phy::handle_dl_schedule(void)
{
    LTE_fdd_enb_user_mgr* user_mgr = LTE_fdd_enb_user_mgr::get_instance();
    LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT *dl_sched = new LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT;
    dl_sched->dl_allocations.N_alloc = 0;
    dl_sched->ul_allocations.N_alloc = 0;
    dl_sched->current_tti            = dl_current_tti;
    user_mgr->get_dl_sched(dl_sched);

    boost::mutex::scoped_lock lock(dl_sched_mutex);

    if(dl_sched->dl_allocations.N_alloc ||
       dl_sched->ul_allocations.N_alloc)
    {
        interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_INFO,
                                  LTE_FDD_ENB_DEBUG_LEVEL_PHY,
                                  __FILE__,
                                  __LINE__,
                                  "Received PDSCH schedule from MAC CURRENT_TTI:MAC=%u,PHY=%u N_dl_allocs=%u N_ul_allocs=%u",
                                  dl_sched->current_tti,
                                  dl_current_tti,
                                  dl_sched->dl_allocations.N_alloc,
                                  dl_sched->ul_allocations.N_alloc);
        memcpy(&dl_schedule[dl_sched->current_tti%10], dl_sched, sizeof(LTE_FDD_ENB_DL_SCHEDULE_MSG_STRUCT));
        // Update ul_schedule status
        if(dl_sched->dl_allocations.N_alloc)
        {
            memcpy(&ul_schedule[(dl_sched->current_tti%10+4)%10].decodes, &dl_sched->dl_allocations, sizeof(LIBLTE_PHY_PDCCH_STRUCT));
        }else if(dl_sched->ul_allocations.N_alloc){
            memcpy(&ul_schedule[(dl_sched->current_tti%10+4)%10].decodes, &dl_sched->ul_allocations, sizeof(LIBLTE_PHY_PDCCH_STRUCT));
        }
    }

    late_subfr = false;
}
void LTE_fdd_enb_phy::handle_ul_schedule(void)
{   
    LTE_fdd_enb_user_mgr* user_mgr = LTE_fdd_enb_user_mgr::get_instance();
    boost::mutex::scoped_lock lock(ul_sched_mutex);

    //user_mgr->get_ul_sched(&ul_schedule[ul_sched->current_tti%10]);

    // if(ul_sched->current_tti                    < ul_current_tti &&
    //    (ul_current_tti - ul_sched->current_tti) < (LTE_FDD_ENB_CURRENT_TTI_MAX/2))
    // {
    //     interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_ERROR,
    //                               LTE_FDD_ENB_DEBUG_LEVEL_PHY,
    //                               __FILE__,
    //                               __LINE__,
    //                               "Late UL subframe from MAC:%u, PHY is currently on %u",
    //                               ul_sched->current_tti,
    //                               ul_current_tti);
    // }else{
    //     if(ul_sched->decodes.N_alloc)
    //     {
    //         interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_INFO,
    //                                   LTE_FDD_ENB_DEBUG_LEVEL_PHY,
    //                                   __FILE__,
    //                                   __LINE__,
    //                                   "Received PUSCH schedule from MAC CURRENT_TTI:MAC=%u,PHY=%u N_ul_decodes=%u",
    //                                   ul_sched->current_tti,
    //                                   ul_current_tti,
    //                                   ul_sched->decodes.N_alloc);
    //     }

    //     memcpy(&ul_schedule[ul_sched->current_tti%10], ul_sched, sizeof(LTE_FDD_ENB_UL_SCHEDULE_MSG_STRUCT));
    // }
}

void* LTE_fdd_enb_phy::process_dl(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf)
{
    //LTE_FDD_ENB_RADIO_TX_BUF_STRUCT*      tx_buf = (LTE_FDD_ENB_RADIO_TX_BUF_STRUCT*)_tx_buf;
    LTE_FDD_ENB_READY_TO_SEND_MSG_STRUCT  rts;
    uint32                                p;
    uint32                                i;
    uint32                                j;
    uint32                                last_prb = 0;
    uint32                                act_noutput_items;
    uint32                                sfn   = dl_current_tti/10;
    uint32                                subfn = dl_current_tti%10;
    uint32                                rand_num;
    bool                                  sys_exist = false;
    

    srand(time(NULL));
    // Initialize the output to all zeros
    for(p=0; p<sys_info.N_ant; p++)
    {
        for(i=0; i<14; i++)
        {
            for(j=0; j<LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++)
            {
                //dl_subframe.tx_symb_re[p][i][j] = 0;
                //dl_subframe.tx_symb_im[p][i][j] = 0;
                rand_num = rand()%2;
                if(rand_num == 0)
                   dl_subframe.tx_symb_re[p][i][j] = 1/sqrt(2);
                else
                   dl_subframe.tx_symb_re[p][i][j] = -1/sqrt(2);
                
                rand_num = rand()%2;
                if(rand_num == 0)
                   dl_subframe.tx_symb_im[p][i][j] = 1/sqrt(2);
                else
                   dl_subframe.tx_symb_im[p][i][j] = -1/sqrt(2);
            }
        }
    }
    dl_subframe.num = subfn;

    // Handle PSS and SSS
    if(0 == dl_subframe.num ||
       5 == dl_subframe.num)
    {
        liblte_phy_map_pss(phy_struct,
                           &dl_subframe,
                           sys_info.N_id_2,
                           sys_info.N_ant);
        liblte_phy_map_sss(phy_struct,
                           &dl_subframe,
                           sys_info.N_id_1,
                           sys_info.N_id_2,
                           sys_info.N_ant);
    }
    // Handle CRS
    liblte_phy_map_crs(phy_struct,
                       &dl_subframe,
                       sys_info.N_id_cell,
                       sys_info.N_ant);
    
    // Handle PBCH
    if(0 == dl_subframe.num)
    {
        sys_info.mib.sfn_div_4 = sfn/4;
        liblte_rrc_pack_bcch_bch_msg(&sys_info.mib,
                                     &dl_rrc_msg);
        liblte_phy_bch_channel_encode(phy_struct,
                                      dl_rrc_msg.msg,
                                      dl_rrc_msg.N_bits,
                                      sys_info.N_id_cell,
                                      sys_info.N_ant,
                                      &dl_subframe,
                                      sfn);
        sys_exist=true;
    }
    // Handle SIB data
    pdcch.N_alloc = 0;
    if(5 == dl_subframe.num &&
       0 == (sfn % 2))
    {
        // SIB-1
        memcpy(&pdcch.alloc[pdcch.N_alloc], &sys_info.sib1_alloc, sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
        liblte_phy_get_tbs_mcs_and_n_prb_for_dl(pdcch.alloc[pdcch.N_alloc].msg.N_bits,
                                                dl_subframe.num,
                                                sys_info.N_rb_dl,
                                                pdcch.alloc[pdcch.N_alloc].rnti,
                                                &pdcch.alloc[pdcch.N_alloc].tbs,
                                                &pdcch.alloc[pdcch.N_alloc].mcs,
                                                &pdcch.alloc[pdcch.N_alloc].N_prb);
        pdcch.alloc[pdcch.N_alloc].rv_idx = (uint32)ceilf(1.5 * ((sfn / 2) % 4)) % 4; //36.321 section 5.3.1
        pdcch.N_alloc++;
        sys_exist = true;
    }
    if((0 * sys_info.si_win_len)%10   <= dl_subframe.num &&
       (1 * sys_info.si_win_len)%10   >  dl_subframe.num &&
       ((0 * sys_info.si_win_len)/10) == (sfn % sys_info.si_periodicity_T))
    {
        // SIs in 1st scheduling info list entry
        //memcpy(&pdcch.alloc[pdcch.N_alloc], &sys_info.sib_alloc[0], sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
        // FIXME: This was a hack to allow SIB2 decoding with 1.4MHz BW due to overlap with MIB
        //if(LIBLTE_SUCCESS == liblte_phy_get_tbs_mcs_and_n_prb_for_dl(pdcch.alloc[pdcch.N_alloc].msg.N_bits,
        //                                                             dl_subframe.num,
        //                                                             sys_info.N_rb_dl,
        //                                                             pdcch.alloc[pdcch.N_alloc].rnti,
        //                                                             &pdcch.alloc[pdcch.N_alloc].tbs,
        //                                                             &pdcch.alloc[pdcch.N_alloc].mcs,
        //                                                             &pdcch.alloc[pdcch.N_alloc].N_prb))
        //{
        //    pdcch.N_alloc++;
        //    sys_exist=true;
        //}
    }
    for(i=1; i<sys_info.sib1.N_sched_info; i++)
    {
        if(0                              != sys_info.sib_alloc[i].msg.N_bits &&
          (i * sys_info.si_win_len)%10   == dl_subframe.num                  &&
          ((i * sys_info.si_win_len)/10) == (sfn % sys_info.si_periodicity_T))
        {
            memcpy(&pdcch.alloc[pdcch.N_alloc], &sys_info.sib_alloc[i], sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
            liblte_phy_get_tbs_mcs_and_n_prb_for_dl(pdcch.alloc[pdcch.N_alloc].msg.N_bits,
                                                    dl_subframe.num,
                                                    sys_info.N_rb_dl,
                                                    pdcch.alloc[pdcch.N_alloc].rnti,
                                                    &pdcch.alloc[pdcch.N_alloc].tbs,
                                                    &pdcch.alloc[pdcch.N_alloc].mcs,
                                                    &pdcch.alloc[pdcch.N_alloc].N_prb);
            pdcch.N_alloc++;
            sys_exist=true;
        }
    }
    
        
    if(sys_exist == false)
    {
        handle_dl_schedule();
        fprintf(stderr, "\n%d no system information\n",dl_current_tti);
    }
    /*********************************************************************************************************************************************/
    dl_sched_mutex.lock();
    if(dl_schedule[dl_current_tti%10].current_tti == dl_current_tti)
    {
        for(i=0; i<dl_schedule[subfn].dl_allocations.N_alloc; i++)
        {
            memcpy(&pdcch.alloc[pdcch.N_alloc], &dl_schedule[subfn].dl_allocations.alloc[i], sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
            pdcch.N_alloc++;
        }
        for(i=0; i<dl_schedule[subfn].ul_allocations.N_alloc; i++)
        {
            memcpy(&pdcch.alloc[pdcch.N_alloc], &dl_schedule[subfn].ul_allocations.alloc[i], sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
            pdcch.N_alloc++;
        }
    }else{
                  
    }
    dl_sched_mutex.unlock();
    
    
    if(last_prb > phy_struct->N_rb_dl)
    {
        
    }else{
        liblte_phy_pdcch_channel_encode(phy_struct,
                                        &pcfich,
                                        &phich[subfn],
                                        &pdcch,
                                        sys_info.N_id_cell,
                                        sys_info.N_ant,
                                        liblte_rrc_phich_resource_num[sys_info.mib.phich_config.res],
                                        sys_info.mib.phich_config.dur,
                                        &dl_subframe);
        
        if(0 != pdcch.N_alloc)
        {
            // SDR_demo 
            liblte_phy_pdsch_channel_encode(phy_struct,
                                            &pdcch,
                                            sys_info.N_id_cell,
                                            sys_info.N_ant,
                                            &dl_subframe);
        }
        else{
            fprintf(stderr, "-----%d tti, no scheduling... -----\n",dl_current_tti);
            //getchar();
        }
        // Clear PHICH
        for(i=0; i<25; i++)
        {
            for(j=0; j<8; j++)
            {
                phich[subfn].present[i][j] = false;
            }
        }
    }
    
    for(p=0; p<sys_info.N_ant; p++)
    {
        liblte_phy_create_dl_subframe(phy_struct,
                                      &dl_subframe,
                                      p,
                                      &tx_buf->i_buf[p][0],
                                      &tx_buf->q_buf[p][0]);
    }
    tx_buf->current_tti = dl_current_tti%10;

    // Update current TTI
    dl_current_tti = (dl_current_tti + 1) % (LTE_FDD_ENB_CURRENT_TTI_MAX + 1);

    // // Test //
    // cerr<<"--------- My Test ---------"<<endl;
    // cerr<<setw(20)<<"subframe "<<tx_buf->current_tti<<endl;
    // LTE_FDD_ENB_RADIO_RX_BUF_STRUCT rx_test;
    // LIBLTE_PHY_SUBFRAME_STRUCT      rx_subframe;
    // LIBLTE_PHY_PCFICH_STRUCT        rx_pcfich;
    // LIBLTE_PHY_PHICH_STRUCT         rx_phich;
    // LIBLTE_PHY_PDCCH_STRUCT         rx_pdcch;
    // LIBLTE_BIT_MSG_STRUCT           rx_rrc_msg;
    // rx_test.current_tti = tx_buf->current_tti;
    // for(i=0; i<phy_struct->N_samps_per_subfr; i++)
    // {
    //     rx_test.i_buf[i] = tx_buf->i_buf[0][i];
    //     rx_test.q_buf[i] = tx_buf->q_buf[0][i];
    // }
    // liblte_phy_get_dl_subframe_and_ce(phy_struct,
    //                                   rx_test.i_buf,
    //                                   rx_test.q_buf,
    //                                   0-rx_test.current_tti*phy_struct->N_samps_per_subfr,
    //                                   rx_test.current_tti,
    //                                   sys_info.N_id_cell,
    //                                   4,
    //                                   &rx_subframe);
    // if(rx_test.current_tti!=0 && rx_test.current_tti!=5)
    // {
    //     liblte_phy_pdcch_channel_decode(phy_struct,
    //                                     &rx_subframe,
    //                                     sys_info.N_id_cell,
    //                                     1,
    //                                     liblte_rrc_phich_resource_num[sys_info.mib.phich_config.res],
    //                                     sys_info.mib.phich_config.dur,
    //                                     &rx_pcfich,
    //                                     &rx_phich,
    //                                     &rx_pdcch);
    //     uint32 pdcch_idx = 0;
        
    //     liblte_phy_pdsch_channel_decode(phy_struct,
    //                                     &rx_subframe,
    //                                     &rx_pdcch.alloc[pdcch_idx], ///Fixed by Chia-Hao Chang, it has problems... [0] or [2]
    //                                     rx_pdcch.N_symbs,
    //                                     sys_info.N_id_cell,
    //                                     1,
    //                                     rx_rrc_msg.msg,
    //                                     &rx_rrc_msg.N_bits);
    //     uint8* tmp = rx_rrc_msg.msg;
    //     cerr<<setw(30)<<"Message : "<<bits_2_value_char(&tmp, 8)<<bits_2_value_char(&tmp, 8)<<endl;
    //     getchar();
    // }
}

int LTE_fdd_enb_phy::get_sample_rate()
{
    if(!started)
    {
        switch(var_map_int64[LTE_FDD_ENB_PARAM_DL_BW])
        {
        case LIBLTE_RRC_DL_BANDWIDTH_100:
            // Intentional fall-thru
        case LIBLTE_RRC_DL_BANDWIDTH_75:
            fs                = 30720000;
            N_samps_per_subfr = LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_50:
            fs                = 15360000;
            N_samps_per_subfr = LIBLTE_PHY_N_SAMPS_PER_SUBFR_15_36MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_25:
            fs                = 7680000;
            N_samps_per_subfr = LIBLTE_PHY_N_SAMPS_PER_SUBFR_7_68MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_15:
            fs                = 3840000;
            N_samps_per_subfr = LIBLTE_PHY_N_SAMPS_PER_SUBFR_3_84MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_6:
            // Intentional fall-thru
        default:
            fs                = 1920000;
            N_samps_per_subfr = LIBLTE_PHY_N_SAMPS_PER_SUBFR_1_92MHZ;
            break;
        }
    }
    return fs;
}

// void LTE_fdd_enb_phy::handle_mac_msg(LTE_FDD_ENB_MESSAGE_STRUCT *msg)
// {
//     if(LTE_FDD_ENB_DEST_LAYER_PHY == msg->dest_layer ||
//        LTE_FDD_ENB_DEST_LAYER_ANY == msg->dest_layer)
//     {
//         switch(msg->type)
//         {
//         case LTE_FDD_ENB_MESSAGE_TYPE_DL_SCHEDULE:
//             handle_dl_schedule(&msg->msg.dl_schedule);
//             delete msg;
//             break;
//         case LTE_FDD_ENB_MESSAGE_TYPE_UL_SCHEDULE:
//             handle_ul_schedule(&msg->msg.ul_schedule);
//             delete msg;
//             break;
//         default:
//             interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_WARNING,
//                                       LTE_FDD_ENB_DEBUG_LEVEL_PHY,
//                                       __FILE__,
//                                       __LINE__,
//                                       "Received invalid message %s",
//                                       LTE_fdd_enb_message_type_text[msg->type]);
//             delete msg;
//             break;
//         }
//     }else{
//         interface->send_debug_msg(LTE_FDD_ENB_DEBUG_TYPE_ERROR,
//                                   LTE_FDD_ENB_DEBUG_LEVEL_PHY,
//                                   __FILE__,
//                                   __LINE__,
//                                   "Received message for invalid layer %s",
//                                   LTE_fdd_enb_dest_layer_text[msg->dest_layer]);
//         delete msg;
//     }
// }

void LTE_fdd_enb_phy::cleanup(void)
{
    boost::mutex::scoped_lock lock(phy_instance_mutex);

    if(NULL != instance)
    {
        delete instance;
        instance = NULL;
    }
}

void LTE_fdd_enb_phy::stop(void)
{
    if(started)
    {
        started = false;

        liblte_phy_ul_cleanup(phy_struct);
        liblte_phy_cleanup(phy_struct);

        delete mac_comm_msgq;
    }
}

void LTE_fdd_enb_phy::radio_interface(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf, LTE_FDD_ENB_RADIO_RX_BUF_STRUCT *rx_buf)
{
    // the member function of class need to transform into function object by using bind
    // created by Chia-Hao Chang
    boost::thread dl_thread(boost::bind(&LTE_fdd_enb_phy::process_dl, this, tx_buf));
    boost::thread ul_thread(boost::bind(&LTE_fdd_enb_phy::process_ul, this, rx_buf));

    dl_thread.join();
    ul_thread.join();

}

void LTE_fdd_enb_phy::radio_interface(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *tx_buf)
{
    boost::thread dl_thread(boost::bind(&LTE_fdd_enb_phy::process_dl, this, tx_buf));
    dl_thread.join();
}

void* LTE_fdd_enb_phy::process_ul(LTE_FDD_ENB_RADIO_RX_BUF_STRUCT *rx_buf)
{
    //LTE_FDD_ENB_RADIO_RX_BUF_STRUCT* rx_buf = (LTE_FDD_ENB_RADIO_RX_BUF_STRUCT*)_rx_buf;
    LTE_fdd_enb_user_mgr* user_mgr            = LTE_fdd_enb_user_mgr::get_instance()  ;

    uint32 N_skipped_subfrs = 0;
    uint32 sfn;
    uint32 i;
    uint32 I_prb_ra;
    uint32 n_group_phich;
    uint32 n_seq_phich;
    uint16 c_rnti;
    float p1_re_0;
    float p1_im_0;
    float n1_re_0;
    float n1_im_0;
    float p1_re_1;
    float p1_im_1;
    float n1_re_1;
    float n1_im_1;

    // Check the received current_tti

    // Fixed by Chia-Hao Chang
    ul_current_tti = rx_buf->current_tti;

    // if(rx_buf->current_tti != ul_current_tti)
    // {
    //     cerr << "rx_buf->current_tti != ul_current_tti" << endl;
    //     if(rx_buf->current_tti > ul_current_tti)
    //     {
    //         N_skipped_subfrs = rx_buf->current_tti - ul_current_tti;
    //     }else{
    //         N_skipped_subfrs = (rx_buf->current_tti + LTE_FDD_ENB_CURRENT_TTI_MAX + 1) - ul_current_tti;
    //     }

    //     // Jump the DL and UL current_tti
    //     dl_current_tti = (dl_current_tti + N_skipped_subfrs) % (LTE_FDD_ENB_CURRENT_TTI_MAX + 1);
    //     ul_current_tti = (ul_current_tti + N_skipped_subfrs) % (LTE_FDD_ENB_CURRENT_TTI_MAX + 1);
    // }
    sfn             = ul_current_tti/10;
    ul_subframe.num = ul_current_tti%10;

    // Handle PRACH
    
    // if((sfn % prach_sfn_mod) == 0) // Check System Frame Number 
    // { 
    //     if((ul_subframe.num % prach_subfn_mod) == prach_subfn_check) // Check Subframe Number
    //     {
    //         if(ul_subframe.num != 0 ||
    //            true            == prach_subfn_zero_allowed)
    //         {
    //             prach_decode.current_tti = ul_current_tti;
    //             liblte_phy_detect_prach(phy_struct,
    //                                     rx_buf->i_buf,
    //                                     rx_buf->q_buf,
    //                                     sys_info.sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset,
    //                                     &prach_decode.num_preambles,
    //                                     prach_decode.preamble,
    //                                     prach_decode.timing_adv);

    //             LTE_fdd_enb_msgq::send(phy_mac_mq,
    //                                    LTE_FDD_ENB_MESSAGE_TYPE_PRACH_DECODE,
    //                                    LTE_FDD_ENB_DEST_LAYER_MAC,
    //                                    (LTE_FDD_ENB_MESSAGE_UNION *)&prach_decode,
    //                                    sizeof(LTE_FDD_ENB_PRACH_DECODE_MSG_STRUCT));
    //         }
    //     }
    // }

    // Handle PUSCH & PUCCH
    ul_sched_mutex.lock();
    if(0 != ul_schedule[ul_subframe.num].decodes.N_alloc)
    {
        cerr << "There is allocation : " << ul_schedule[ul_subframe.num].decodes.alloc[0].rnti << endl;

        if(LIBLTE_SUCCESS == liblte_phy_get_ul_subframe(phy_struct,
                                                        rx_buf->i_buf,
                                                        rx_buf->q_buf,
                                                        &ul_subframe))
        {
            for(i=0; i<ul_schedule[ul_subframe.num].decodes.N_alloc; i++)
            {
                // Handle DLSCH's PUCCH HARQ report
                // Implemented by Chia-Hao Chang
                if(ul_schedule[ul_subframe.num].decodes.alloc[i].chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
                {
                    liblte_phy_pucch_formats_1_1a_1b_decode(phy_struct,
                                                            1, // one-bit
                                                            sys_info.N_ant,
                                                            1, //delta_PUCCH_shift, fixed
                                                            sys_info.N_id_cell,
                                                            0,
                                                            &p1_re_0,
                                                            &p1_im_0,
                                                            &n1_re_0,
                                                            &n1_im_0,
                                                            &ul_subframe);
                    liblte_phy_pucch_formats_1_1a_1b_decode(phy_struct,
                                                            1, // one-bit 
                                                            sys_info.N_ant,
                                                            1, //delta_PUCCH_shift, fixed
                                                            sys_info.N_id_cell,
                                                            1,
                                                            &p1_re_1,
                                                            &p1_im_1,
                                                            &n1_re_1,
                                                            &n1_im_1,
                                                            &ul_subframe);
                    //  theoretical value 96+j0, find which has min distance
                    float dist_to_nack = (p1_re_0+p1_re_1-96)*(p1_re_0+p1_re_1-96)+(p1_im_0+p1_im_1)*(p1_im_0+p1_im_1);
                    float dist_to_ack  = (n1_re_0+n1_re_1-96)*(n1_re_0+n1_re_1-96)+(n1_im_0+n1_im_1)*(n1_im_0+n1_im_1);
                    
                    // Need to determine new transmission or retransmission
                    //user_mgr->find_ul_user(ul_current_tti%10, &c_rnti);

                    if(dist_to_nack < dist_to_ack) // NACK
                    {
                        cerr << "NACK from "<< ul_schedule[ul_subframe.num].decodes.alloc[i].rnti << endl;
                        if((ul_current_tti+4)%10 == 5)
                        {
                            user_mgr->set_dl_sched(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, 
                                                   (ul_current_tti+5), 
                                                   false, 
                                                   LIBLTE_PHY_CHAN_TYPE_DLSCH);
                        }else{
                            user_mgr->set_dl_sched(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, 
                                                   (ul_current_tti+4), 
                                                   false, 
                                                   LIBLTE_PHY_CHAN_TYPE_DLSCH);
                        }
                    }else{
                        cerr << "ACK from "<< ul_schedule[ul_subframe.num].decodes.alloc[i].rnti << endl;
                        if((ul_current_tti+4)%10 == 5)
                        {
                            user_mgr->set_dl_sched(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, 
                                                   (ul_current_tti+5), 
                                                   false, 
                                                   LIBLTE_PHY_CHAN_TYPE_DLSCH);
                        }else{
                            user_mgr->set_dl_sched(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, 
                                                   (ul_current_tti+4), 
                                                   true, 
                                                   LIBLTE_PHY_CHAN_TYPE_DLSCH);
                        }
                    }   

                }else{
                    // Determine PHICH indecies
                    I_prb_ra      = ul_schedule[ul_subframe.num].decodes.alloc[i].prb[0][0];
                    n_group_phich = I_prb_ra % phy_struct->N_group_phich;
                    n_seq_phich   = (I_prb_ra/phy_struct->N_group_phich) % (2*phy_struct->N_sf_phich);

                    // Attempt decode
                    if(LIBLTE_SUCCESS == liblte_phy_pusch_channel_decode(phy_struct,
                                                                         &ul_subframe,
                                                                         &ul_schedule[ul_subframe.num].decodes.alloc[i],
                                                                         sys_info.N_id_cell,
                                                                         1,
                                                                         pusch_decode.msg.msg,
                                                                         &pusch_decode.msg.N_bits))
                    {
                        fprintf(stderr, "liblte_phy_pusch_channel_decode succeed...\n");
                        pusch_decode.current_tti = ul_current_tti;
                        pusch_decode.rnti        = ul_schedule[ul_subframe.num].decodes.alloc[i].rnti;

                        user_mgr->print_msg(&pusch_decode.msg, ul_schedule[ul_subframe.num].decodes.alloc[i].rv_idx);

                        // Add ACK to PHICH
                        phich[(ul_subframe.num + 4) % 10].present[n_group_phich][n_seq_phich] = true;
                        phich[(ul_subframe.num + 4) % 10].b[n_group_phich][n_seq_phich]       = 1;

                        // Receive UL Message 
                        user_mgr->receive_msg(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, &pusch_decode);

                        // Update Next Scheduling
                        user_mgr->set_dl_sched(ul_schedule[ul_subframe.num].decodes.alloc[i].rnti, 
                                               (ul_current_tti+8)%(LTE_FDD_ENB_CURRENT_TTI_MAX + 1), // if ACK
                                               true, 
                                               LIBLTE_PHY_CHAN_TYPE_ULSCH);

                    }else{
                        // Add NACK to PHICH
                        fprintf(stderr, "liblte_phy_pusch_channel_decode failed...\n");
                        phich[(ul_subframe.num + 4) % 10].present[n_group_phich][n_seq_phich] = true;
                        phich[(ul_subframe.num + 4) % 10].b[n_group_phich][n_seq_phich]       = 0;
                    }
                }
            }
        }
    }else{
        cerr << "There isn't allocation" << endl;
    }
    ul_schedule[ul_subframe.num].decodes.N_alloc = 0;
    ul_sched_mutex.unlock();

    // Update counters
    ul_current_tti = (ul_current_tti + 1) % (LTE_FDD_ENB_CURRENT_TTI_MAX + 1);
}