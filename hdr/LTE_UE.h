#ifndef __LTE_UE_H__
#define __LTE_UE_H__
	
/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp> 
#include <boost/format.hpp>

#include <iostream>
#include <stdlib.h>
#include <numeric>
#include <complex>
#include <chrono>
#include <vector>
#include <string>
#include <math.h>  
#include <queue>                               
#include <map>                           

#include "liblte_mcc_mnc_list.h" 
#include "liblte_phy.h" 

#include "LTE_message_queue.h"                             
#include "LTE_file.h"

                               

                              
using namespace std;
/*******************************************************************************
                              DEFINES
*******************************************************************************/
#define LTE_FDD_DL_FS_SAMP_BUF_SIZE       (LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ*10)
#define LTE_FDD_DL_FS_SAMP_BUF_NUM_FRAMES (10)
#define LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX 10
#define LTE_FDD_ENB_CURRENT_TTI_MAX (LIBLTE_PHY_SFN_MAX*10 + 9)

// Configurable Parameters
#define FS_PARAM "fs"

/*******************************************************************************
                              FORWARD DECLARATIONS
*******************************************************************************/

/*******************************************************************************
                              STATIC VARIABLES
*******************************************************************************/
static float ZERO_SIGNAL[1920]          = {0.0};
static float TX_SIGNAL_RE[1920]         = {0.0};
static float TX_SIGNAL_IM[1920]         = {0.0};
static float TX_OFDM_SYMBOL_RE[19200]   = {0.0};
static float TX_OFDM_SYMBOL_IM[19200]   = {0.0};
static uint32 samp_nu                   = 0;
static int32  cfi_failed_nu             = 0;
static bool   continous_cfi_failed      = false;
static int32 recv_more                  = 0;

/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/
/*  Fixed by Chia-Hao Chang 2014/11/22 */
typedef struct{
    float i_buf[4][/*LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ/**/ LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*80 ];
    float q_buf[4][/*LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ/**/ LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*80 ];
    uint16 current_frame;   
}LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ;

typedef struct{
    float  i_buf[4][LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    float  q_buf[4][LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    uint16 current_tti;
}LTE_FDD_ENB_RADIO_TX_BUF_STRUCT;

typedef struct{
    float  i_buf[LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    float  q_buf[LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    uint16 current_tti;
}LTE_FDD_ENB_RADIO_RX_BUF_STRUCT;

//typedef boost::shared_ptr<LTE_fdd_dl_fs_samp_buf> LTE_fdd_dl_fs_samp_buf_sptr;

typedef enum{
    LTE_FDD_DL_FS_IN_SIZE_INT8 = 0,
    LTE_FDD_DL_FS_IN_SIZE_GR_COMPLEX,
}LTE_FDD_DL_FS_IN_SIZE_ENUM;

typedef enum{
    LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH = 0,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC,
}LTE_FDD_DL_FS_SAMP_BUF_STATE_ENUM;

typedef enum{
    CELL_SEARCH_STATE=0,
    PBCH_DECODE_STATE,
    PDCCH_DECODE_STATE,
    PDSCH_DECODE_STATE,
    PUSCH_ENCODE_STATE,
    PUCCH_REPORT_STATE,
    PHICH_DECODE_STATE,
    WATING_STATE,
    UNDEFINED_STATE,
}RECV_STATE;

/*******************************************************************************
                              FUNCTIONS
*******************************************************************************/

void ul_offset_calculation(LIBLTE_PHY_STRUCT               *phy_struct,
                           LIBLTE_PHY_SUBFRAME_STRUCT      *subframe,
                           LIBLTE_PHY_ALLOCATION_STRUCT    **alloc, 
                           uint32                           num_users,
                           float                           *freq_offset_u
                          );
void ul_cfo_sco_compensation(LIBLTE_PHY_STRUCT               *phy_struct,
                             float                            NCO_phase,
                             float                            freq_offset_u,
                             float                           *i_buf,
                             float                           *q_buf,
                             float                           *compensate_i_buf,
                             float                           *compensate_q_buf
                            );

//class

class Radio
{
public:    
    static Radio* get_instance(void);
    void* recv_to_buffer(void* inputs);
    void* send_from_buffer(float* i_data, float* q_data, uint32 len);

    // Message Queue
    void* recv_from_mq(void* inputs);

    // Uplink mutex
    void* process_ul_mq(void);

    // Start/Stop
    boost::mutex start_mutex;
    bool         started;

    void init();
    void init_usrp(double rate_0, double freq_0, double gain_0, bool clock_0,
                   double rate_1, double freq_1, double gain_1, bool clock_1 ) ;

    // buffer
    std::vector<std::complex<float> >   send_buff; 
    std::vector<std::complex<float> >   recv_buff;

    //Radio thread
    void             radio_thread_func(void* inputs);
    pthread_t        radio_thread;
    uhd::time_spec_t next_tx_ts;
    std::string      clock_source;
    uint32           N_ant;
    uint32           N_tx_samps;
    uint32           N_rx_samps;
    uint32           N_samps_per_subfr;
    uint32           fs;
    uint32           tx_gain;
    uint32           rx_gain;
    uint16           next_tx_current_tti;

    // Parameter
    uint16           external_c_rnti;

    //static  Radio* instance;
    float            sampling_freq;


    uhd::tx_streamer::sptr              tx_stream;
    uhd::rx_streamer::sptr              rx_stream;
private:
    static  Radio* instance;
    Radio();
    ~Radio();
    
    uhd::usrp::multi_usrp::sptr         usrp_0;
    uhd::usrp::multi_usrp::sptr         usrp_1;
    

    uhd::device_addr_t  hint;
    uhd::device_addrs_t devs;


    string  filename;
    string  subdev;
    string  args;
    string  ant;

    double  rate; 
    double  freq;
    double  gain;
    double  thres;

    // metadata
    uhd::tx_metadata_t  tx_md;
    uhd::rx_metadata_t  rx_md;

};


class LTE_fdd_dl_fs_samp_buf
{
public:
    static LTE_fdd_dl_fs_samp_buf* get_instance(void);

    double  carrier_freq;

    void    reset();
    int32   work(LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ frame);
    int32   handle_dl(float* i_recv_buf, float* q_recv_buf, int len);
    uint32  mq_execute(float* i_data, float* q_data, uint32 len);


    /*** Decode Process ***/
     int32  execute(float* i_data, float* q_data, uint32 len);
    uint32  cell_search(float* i_data, float* q_data, uint32 len, uint32* neg_offset);
     int32  pbch_decoding(float* i_data, float* q_data, uint32 len);
    uint32  sib1_decoding(float* i_data, float* q_data, uint32 len);
    uint32  dlsch_decoding(float* i_data, float* q_data, uint32 len, uint32 subfr_num);
    uint32  pucch_encoding(float* i_data, float* q_data, uint32 subfr_num);
    uint32  ulsch_encoding(float* i_data, float* q_data, uint32 subfr_num);
    uint32  phich_decoding(float* i_data, float* q_data, uint32 len, uint32 subfr_num);

    uint32  tracking(float* i_data, float* q_data, uint32 len);
    void    phich_channel_decode(LIBLTE_PHY_STRUCT*                phy_struct,
                                 LIBLTE_PHY_ALLOCATION_STRUCT*     alloc,
                                 LIBLTE_PHY_SUBFRAME_STRUCT        *subframe,
                                 uint32                            N_id_cell,
                                 float                             phich_res,
                                 LIBLTE_RRC_PHICH_DURATION_ENUM    phich_dur,
                                 uint8                             *ACK_or_NACK);

    void    go_next_state();
    void    Farrow_Interpolator(float *SCO_out,
                                float *SCO_in, 
                                float Tnew_div_Torig, 
                                float initial, 
                                uint64 seq_length); 
    void    ul_process();
    uint8   bits_2_value_char(uint8 **bits, uint32 N_bits)
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
    void value_2_bits(int16   value,
                      uint8  **bits,
                      uint32   N_bits)
    {
        uint32 i;
    
        for(i=0; i<N_bits; i++)
        {
            (*bits)[i] = (value >> (N_bits-i-1)) & 0x1;
        }
        *bits += N_bits;
    }
    int16 bits_2_value(uint8  **bits,
                        uint32   N_bits)
    {
        int16 value = 0;
        uint32 i;
    
        for(i=0; i<N_bits; i++)
        {
            value |= (*bits)[i] << (N_bits-i-1);
        }
        *bits += N_bits;
    
        return(value);
    }

    int16                           timing_advance;
    
    uint16                          C_RNTI;
    uint32                          sfn;
    uint32                          current_tti;
    RECV_STATE                      recv_state;

    /*** Allocation Information ***/
    LIBLTE_PHY_ALLOCATION_STRUCT    Alloc_Info;
    uint32                          time_to_op; // time to operate uplink or recv harq 
    bool                            get_ul_init(void){ return ul_init; }
    void                            set_ul_msg(void);
    bool                            first_tran;

    // TS 36.321 5.3.2.2
    std::vector<LIBLTE_BIT_MSG_STRUCT> message;
    uint32 message_idx;
    class HARQ_process   
    {
    public:
        friend class LTE_fdd_dl_fs_samp_buf;
        HARQ_process():CURRENT_TX_NB(0), retransmission(false), NDI(false){}
        bool is_toggled(bool recv_ndi){ 
            if(NDI != recv_ndi)
            {
                retransmission=false;
                return true;
            }else{
                retransmission=true;
                return false;
            }
        }
        void    set_TX_NB(uint32 tx_nb){ CURRENT_TX_NB=tx_nb; }
        void    set_NDI(uint8 recv_ndi){ NDI=recv_ndi; }
        void    set_RV(uint32 rv){ CURRENT_IRV=rv; }
        uint32  get_RV(){ return CURRENT_IRV; }
        bool    retransmission;
    private:
        uint32 CURRENT_TX_NB; // the number of transmissions that have taken place in the buffer
        uint32 HARQ_FEEDBACK; // indicate the HARQ feedback for PDU currently in the buffer
        uint32 CURRENT_IRV;   // index into the sequence of redundancy version, up-dated modulo 4   
        bool   NDI;    
    };
    HARQ_process                                harq_proc[8];
    HARQ_process                                ul_harq_proc[8];
    LIBLTE_PHY_PDCCH_STRUCT                     pdcch;
    LIBLTE_PHY_PDCCH_STRUCT                     prev_alloc;

    // Uplink thread mutex 
    boost::mutex    ul_tx_mutex;

    // USRP control 
    uint32          more_samps;

    
private:
    //friend LTE_FDD_DL_FS_API LTE_fdd_dl_fs_samp_buf_sptr LTE_fdd_dl_fs_make_samp_buf(size_t in_size_val);
    static LTE_fdd_dl_fs_samp_buf *instance;
    LTE_fdd_dl_fs_samp_buf(size_t);
    ~LTE_fdd_dl_fs_samp_buf();

    // Input parameters
    LTE_FDD_DL_FS_IN_SIZE_ENUM          in_size;


    // Uplink
    LIBLTE_PHY_SUBFRAME_STRUCT  ul_subframe;
    uint8                       ack_or_nack;
    uint8                       tx_information;
    bool                        ul_init;



    // LTE library
    LIBLTE_PHY_STRUCT                          *phy_struct;
    LIBLTE_PHY_COARSE_TIMING_STRUCT             timing_struct;
    LIBLTE_BIT_MSG_STRUCT                       rrc_msg;
    LIBLTE_RRC_MIB_STRUCT                       mib;
    LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT            bcch_dlsch_msg;
    LIBLTE_RRC_PCCH_MSG_STRUCT                  pcch_msg;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT     sib1;
    LIBLTE_PHY_PCFICH_STRUCT                    pcfich;
    LIBLTE_PHY_PHICH_STRUCT                     phich;
    LIBLTE_PHY_FS_ENUM                          fs;

    // Sample buffer
    float  *i_buf;
    float  *q_buf;
    uint32  samp_buf_w_idx;
    uint32  samp_buf_r_idx;
    bool    last_samp_was_i;

    // Variables
    LTE_FDD_DL_FS_SAMP_BUF_STATE_ENUM state;
    float                             phich_res;
    
    uint32                            N_sfr;
    uint32                            N_id_cell;
    uint32                            N_id_1;
    uint32                            N_id_2;
    uint32                            corr_peak_idx;
    uint32                            decoded_chans[LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX];
    uint32                            N_decoded_chans;
    uint8                             N_ant;
    uint8                             prev_si_value_tag;
    bool                              prev_si_value_tag_valid;
    bool                              mib_printed;
    bool                              sib1_printed;
    bool                              sib2_printed;
    bool                              sib3_printed;
    bool                              sib3_expected;
    bool                              sib4_printed;
    bool                              sib4_expected;
    bool                              sib5_printed;
    bool                              sib5_expected;
    bool                              sib6_printed;
    bool                              sib6_expected;
    bool                              sib7_printed;
    bool                              sib7_expected;
    bool                              sib8_printed;
    bool                              sib8_expected;
    bool                              sib_recv;

    // Helpers
    void init(void);
    //void copy_input_to_samp_buf(gr_vector_const_void_star &input_items, int32 ninput_items);
    void freq_shift(uint32 start_idx, uint32 num_samps, float freq_offset);
    void freq_shift_Chang(uint32 start_idx, uint32 num_samps, float freq_offset);
    
    void print_msg(LIBLTE_BIT_MSG_STRUCT *rrc_msg);
    void print_mib(LIBLTE_RRC_MIB_STRUCT *mib);
    void print_sib1(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1);
    void print_sib2(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2);
    void print_sib3(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3);
    void print_sib4(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4);
    void print_sib5(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5);
    void print_sib6(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6);
    void print_sib7(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7);
    void print_sib8(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8);
    //void print_page(LIBLTE_RRC_PAGING_STRUCT *page);
    float low_pass_filter(float JWLSE_cfo);
    void wrap_phase(float *phase_1, float  phase_2);
    void wrap_offset(float *offset_1, float  offset_2);

    void SCO_compensate(LIBLTE_PHY_STRUCT          *phy_struct,
                        uint32                      symbol_offset,
                        uint32                      start_idx,
                        float                       timing_offset,
                        LIBLTE_PHY_SUBFRAME_STRUCT *subframe);

    void Channel_Estimation_for_USRP(LIBLTE_PHY_STRUCT          *phy_struct,
                                     float                      *i_samps,
                                     float                      *q_samps,
                                     uint32                      start_idx,
                                     LIBLTE_PHY_SUBFRAME_STRUCT *subframe);

    void JWLS_for_USRP(LIBLTE_PHY_STRUCT          *phy_struct,
                       float                      *i_samps,
                       float                      *q_samps,
                       uint32                      frame_start_idx,
                       uint8                       subfr_num,
                       uint32                      N_id_cell,
                       uint8                       N_ant,
                       LIBLTE_PHY_SUBFRAME_STRUCT *subframe);
    
    void freq_shift_rcfo(uint32 start_idx, 
                         uint32  num_samps,
                         float  *prev_phase,
                         float   freq_offset);
    
    void find_half_frame_boundaires(LIBLTE_PHY_STRUCT               *phy_struct,
                                    LIBLTE_PHY_COARSE_TIMING_STRUCT *boundary,
                                    uint32                           times);

    LIBLTE_ERROR_ENUM JWLS(LIBLTE_PHY_STRUCT          *phy_struct,
                           float                      *i_samps,
                           float                      *q_samps,
                           uint32                      frame_start_idx,
                           uint8                       subfr_num,
                           uint32                      N_id_cell,
                           uint8                       N_ant,
                           LIBLTE_PHY_SUBFRAME_STRUCT *subframe);


    // Configuration
    void print_config(void);
    void change_config(char *line);
    bool set_fs(char *char_value);
    bool need_config;
    

    fftwf_complex *s2s_in;
    fftwf_complex *s2s_out;
    fftwf_plan     symbs_to_samps_plan;
    fftwf_plan     samps_to_symbs_plan;

    /*RF Front End*/
    float   prev_offset; /// CFO previous phase offset
    float   time_offset; /// SCO previous processing time
    float   if_shift_fft_window;
    float   cfo;
    float   prev_rcfo;
    float   fcfo;
    float   residual_cfo;
    float   sco; /// (-1*cfo)/(2*10^9*FFT_size*(1/sampling_freq))
    float   reg;
    uint32  len_offset;
    uint32  residual_len;

    // Receiving state
    
    // UL system information
    uint32             prach_root_seq_idx       ;
    uint32             prach_preamble_format    ;
    uint32             prach_zczc               ;  
    bool               prach_hs_flag            ;
    uint8              group_assignment_pusch   ;
    bool               group_hopping_enabled    ;
    bool               sequence_hopping_enabled ;
    uint8              cyclic_shift             ;
    uint8              cyclic_shift_dci         ;
};

	
#endif