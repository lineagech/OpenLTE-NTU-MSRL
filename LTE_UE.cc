/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include "LTE_UE.h"
#include "liblte_mac.h"

using namespace std;

/*******************************************************************************
                              DEFINES
*******************************************************************************/

#define COARSE_TIMING_N_SLOTS                    (160)
#define COARSE_TIMING_SEARCH_NUM_SUBFRAMES       ((COARSE_TIMING_N_SLOTS/2)+2)   ///82
#define PSS_AND_FINE_TIMING_SEARCH_NUM_SUBFRAMES (COARSE_TIMING_SEARCH_NUM_SUBFRAMES)
#define SSS_SEARCH_NUM_SUBFRAMES                 (COARSE_TIMING_SEARCH_NUM_SUBFRAMES)
#define BCH_DECODE_NUM_FRAMES                    (2)
#define PDSCH_DECODE_SIB1_NUM_FRAMES             (2)
#define PDSCH_DECODE_SI_GENERIC_NUM_FRAMES       (1)


/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/
#define PI 4*atanf(1)

/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/

// minimum and maximum number of input and output streams
const int32 MIN_IN  = 1;
const int32 MAX_IN  = 1;
const int32 MIN_OUT = 0;
const int32 MAX_OUT = 0;

LTE_fdd_dl_fs_samp_buf* LTE_fdd_dl_fs_samp_buf::instance = NULL;
Radio* Radio::instance = NULL;

/*******************************************************************************
                              EXTERN FUNCTIONS
*******************************************************************************/

/*******************************************************************************
                              CLASS IMPLEMENTATIONS
*******************************************************************************/

int RX_find_fft_size(LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ& radio_frame)
{
    float corr_2048_re    =   0;
    float corr_2048_im    =   0;
    float corr_1024_re    =   0;
    float corr_1024_im    =   0;
    float corr_512_re     =   0;
    float corr_512_im     =   0;
    float corr_256_re     =   0;
    float corr_256_im     =   0;
    float corr_128_re     =   0;
    float corr_128_im     =   0;

    float corr_2048           ;
    float corr_1024           ;
    float corr_512            ;
    float corr_256            ;
    float corr_128            ;
    int i                   ;
    /* FFT size 2048 */
    for(i=0; i<LIBLTE_PHY_N_SAMPS_CP_L_0_30_72MHZ; i++){
        corr_2048_re += (radio_frame.i_buf[0][i]*radio_frame.i_buf[0][i+2048] + radio_frame.q_buf[0][i]*radio_frame.q_buf[0][i+2048]);
        corr_2048_im += (radio_frame.q_buf[0][i]*radio_frame.i_buf[0][i+2048] - radio_frame.i_buf[0][i]*radio_frame.q_buf[0][i+2048]);
    }
    corr_2048 = (corr_2048_re*corr_2048_re + corr_2048_im*corr_2048_im)/160;

    /* FFT size 1024 */
    for(i=0; i<LIBLTE_PHY_N_SAMPS_CP_L_0_15_36MHZ; i++){
        corr_1024_re += (radio_frame.i_buf[0][i]*radio_frame.i_buf[0][i+1024] + radio_frame.q_buf[0][i]*radio_frame.q_buf[0][i+1024]);
        corr_1024_im += (radio_frame.q_buf[0][i]*radio_frame.i_buf[0][i+1024] - radio_frame.i_buf[0][i]*radio_frame.q_buf[0][i+1024]);
    }
    corr_1024 = (corr_1024_re*corr_1024_re + corr_1024_im*corr_1024_im)/80;

    /* FFT size 512 */
    for(i=0; i<LIBLTE_PHY_N_SAMPS_CP_L_0_7_68MHZ; i++){
        corr_512_re += (radio_frame.i_buf[0][i]*radio_frame.i_buf[0][i+512] + radio_frame.q_buf[0][i]*radio_frame.q_buf[0][i+512]);
        corr_512_im += (radio_frame.q_buf[0][i]*radio_frame.i_buf[0][i+512] - radio_frame.i_buf[0][i]*radio_frame.q_buf[0][i+512]);
    }
    corr_512 = (corr_512_re*corr_512_re + corr_512_im*corr_512_im)/LIBLTE_PHY_N_SAMPS_CP_L_0_7_68MHZ;

    /* FFT size 256 */
    for(i=0; i<LIBLTE_PHY_N_SAMPS_CP_L_0_3_84MHZ; i++){
        corr_256_re += (radio_frame.i_buf[0][i]*radio_frame.i_buf[0][i+256] + radio_frame.q_buf[0][i]*radio_frame.q_buf[0][i+256]);
        corr_256_im += (radio_frame.q_buf[0][i]*radio_frame.i_buf[0][i+256] - radio_frame.i_buf[0][i]*radio_frame.q_buf[0][i+256]);
    }
    corr_256 = (corr_256_re*corr_256_re + corr_256_im*corr_256_im)/LIBLTE_PHY_N_SAMPS_CP_L_0_3_84MHZ;

    /* FFT size 128 */
    for(i=0; i<LIBLTE_PHY_N_SAMPS_CP_L_0_1_92MHZ; i++){
        corr_128_re += (radio_frame.i_buf[0][i]*radio_frame.i_buf[0][i+128] + radio_frame.q_buf[0][i]*radio_frame.q_buf[0][i+128]);
        corr_128_im += (radio_frame.q_buf[0][i]*radio_frame.i_buf[0][i+128] - radio_frame.i_buf[0][i]*radio_frame.q_buf[0][i+128]);
    }
    corr_128 = (corr_128_re*corr_128_re + corr_128_im*corr_128_im)/LIBLTE_PHY_N_SAMPS_CP_L_0_1_92MHZ;

    unsigned int MAX_CORR = max(corr_2048, max(corr_1024, max(corr_512, max(corr_256, corr_128))));
    return (MAX_CORR==corr_2048) ? 2048 : 
           (MAX_CORR==corr_1024) ? 1024 :
           (MAX_CORR==corr_512)  ? 512  :
           (MAX_CORR==corr_256)  ? 256  :
                                   128  ;
}    
// Sampling Clock Offset
void LTE_fdd_dl_fs_samp_buf::Farrow_Interpolator(float *SCO_out, float* SCO_in, float Tnew_div_Torig, float initial, uint64 seq_length)   
{
    //Farrow interpolator
    uint64      k;
    uint64      m_k;
    float       mu_k;
    for(k=0; k<seq_length; k++)
    {
        m_k     = (uint64)(k*Tnew_div_Torig);
        mu_k    = (float)(k*(Tnew_div_Torig)-m_k);

        if(m_k < 1)
        {
            SCO_out[k] = SCO_in[m_k+2]*(-0.5*mu_k+0.5*pow(mu_k,2)) + SCO_in[m_k+1]*(1.5*mu_k-0.5*pow(mu_k,2)); 
        }
        else if(m_k <= seq_length-3){
            SCO_out[k] = SCO_in[m_k+2]*(-0.5*mu_k+0.5*pow(mu_k,2)) + SCO_in[m_k+1]*(1.5*mu_k-0.5*pow(mu_k,2)) + SCO_in[m_k]*(1-0.5*mu_k-0.5*pow(mu_k,2)) + SCO_in[m_k-1]*(-0.5*mu_k+0.5*pow(mu_k,2));
        }
        else if(m_k == seq_length-2){
            SCO_out[k] = SCO_in[m_k+1]*(1.5*mu_k-0.5*pow(mu_k,2)) + SCO_in[m_k]*(1-0.5*mu_k-0.5*pow(mu_k,2)) + SCO_in[m_k-1]*(-0.5*mu_k+0.5*pow(mu_k,2));
        }
        else if(m_k == seq_length-1){
            SCO_out[k] = SCO_in[m_k]*(1-0.5*mu_k-0.5*pow(mu_k,2)) + SCO_in[m_k-1]*(-0.5*mu_k+0.5*pow(mu_k,2));
        }
        else if(m_k == seq_length){
            SCO_out[k] = SCO_in[m_k-1]*(-0.5*mu_k+0.5*pow(mu_k,2));
        }
        
    }
}

LTE_fdd_dl_fs_samp_buf* LTE_fdd_dl_fs_samp_buf::get_instance(void)
{
    if(NULL == instance)
    {
        instance = new LTE_fdd_dl_fs_samp_buf(128);
    }
    return(instance);
}
             
LTE_fdd_dl_fs_samp_buf::LTE_fdd_dl_fs_samp_buf(size_t fft_size)
:recv_state(CELL_SEARCH_STATE),fs(LIBLTE_PHY_FS_1_92MHZ),need_config(true),message_idx(0)
{
	uint32 i;
	// Initialize the LTE parameters
	//fs = LIBLTE_PHY_FS_1_92MHZ;

	// Initialize the configuration
    //need_config = true;

    // Initialize the sample buffer
    i_buf           = (float *)malloc(LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*50*sizeof(float));
    q_buf           = (float *)malloc(LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*50*sizeof(float));
    samp_buf_w_idx  = 0;
    samp_buf_r_idx  = 0;
    last_samp_was_i = false;


    // Variables
    init();
    cerr<<"intialization...\n";
    N_decoded_chans = 0;
    corr_peak_idx   = 0;
    for(i=0; i<LIBLTE_PHY_N_MAX_ROUGH_CORR_SEARCH_PEAKS; i++)
    {
        timing_struct.freq_offset[i] = 0;
    }

    liblte_phy_init(&phy_struct,
                    fs,
                    LIBLTE_PHY_INIT_N_ID_CELL_UNKNOWN,
                    1,
                    LIBLTE_PHY_N_RB_DL_1_4MHZ,
                    LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
                    liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1]);

    Alloc_Info.chan_type    = UNDEFINED_CHAN_TYPE;
    tx_information          = 'Z';
}
LTE_fdd_dl_fs_samp_buf::~LTE_fdd_dl_fs_samp_buf()
{
    free(i_buf);
    free(q_buf);
}

void LTE_fdd_dl_fs_samp_buf::init(void)
{
	state                   = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
    phich_res               = 0;
    sfn                     = 0;
    N_sfr                   = 0;
    N_ant                   = 0;
    N_id_cell               = 0;
    N_id_1                  = 0;
    N_id_2                  = 0;
    prev_si_value_tag       = 0;
    prev_si_value_tag_valid = false;
    mib_printed             = false;
    sib1_printed            = false;
    sib2_printed            = false;
    sib3_printed            = false;
    sib3_expected           = false;
    sib4_printed            = false;
    sib4_expected           = false;
    sib5_printed            = false;
    sib5_expected           = false;
    sib6_printed            = false;
    sib6_expected           = false;
    sib7_printed            = false;
    sib7_expected           = false;
    sib8_printed            = false;
    sib8_expected           = false;
    ul_init                 = false;
    prev_offset             = 0;

    
    s2s_in              = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*128*2*20);
    s2s_out             = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*128*2*20);
    symbs_to_samps_plan = fftwf_plan_dft_1d(128,
                                            s2s_in,
                                            s2s_out,
                                            FFTW_BACKWARD,
                                            FFTW_MEASURE);
    samps_to_symbs_plan = fftwf_plan_dft_1d(128,
                                            s2s_in,
                                            s2s_out,
                                            FFTW_FORWARD,
                                            FFTW_MEASURE);
    current_tti         = 0;
    cfo                 = 0;
    sco                 = 0;
    reg                 = 0;
    fcfo                = 0;
    len_offset          = 0;
    time_offset         = 0;
    prev_offset         = 0;
    residual_len        = 0;
    prev_rcfo           = 0;
    residual_cfo        = 0;
    if_shift_fft_window = 0;
    C_RNTI              = LIBLTE_MAC_C_RNTI_START;

}

uint32 LTE_fdd_dl_fs_samp_buf::cell_search(float* i_data, float* q_data, uint32 len=9600)
{
    LIBLTE_PHY_COARSE_TIMING_STRUCT  *boundary;
    uint32                            pss_symb;
    uint32                            frame_start_idx;
    float                             pss_thresh;
    float                             freq_offset;
    bool                              subframe_5;

    memcpy(&i_buf[0], &i_data[0], len*sizeof(float));
    memcpy(&q_buf[0], &q_data[0], len*sizeof(float));

    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    
    fprintf(stderr, "coarse timing start is %d...\n",timing_struct.symb_starts[0][0]);
    fprintf(stderr, "fractional freq offset is %10f...\n",timing_struct.freq_offset[0]);
    fcfo = timing_struct.freq_offset[0];
    cfo += timing_struct.freq_offset[0];  /// fcfo
    freq_shift_Chang(0, len, timing_struct.freq_offset[0]);
    
    /*********************************************************************************************************/

    liblte_phy_find_pss_and_fine_timing_Chang(phy_struct,
                                              &i_buf[0],
                                              &q_buf[0],
                                              boundary,
                                              timing_struct.symb_starts[0],
                                              &N_id_2,
                                              &pss_symb,
                                              &pss_thresh,
                                              &freq_offset);

    cfo += freq_offset;
    freq_shift_Chang(0, len, freq_offset);  ///ICFO
    
    timing_struct.freq_offset[0] += freq_offset;
    fprintf(stderr, "Timing start index is %d...\n",timing_struct.symb_starts[0][0]);
    fprintf(stderr, "Compensate ICFO by PSS and freq offset is %10f...\n",freq_offset);
    fprintf(stderr, "N_id_2 is %d...\n",N_id_2);

    liblte_phy_find_sss_Chang(phy_struct,
                              &i_buf[0],
                              &q_buf[0],
                              N_id_2,
                              timing_struct.symb_starts[0],
                              pss_thresh,
                              &N_id_1,
                              &frame_start_idx,
                              subframe_5);
    fprintf(stderr, "\nN_ID_1:%d and N_ID_2:%d...\nframe_start_idx : %d\n", N_id_1, N_id_2, frame_start_idx);
    N_id_cell = N_id_1*3+N_id_2;

    recv_state = PBCH_DECODE_STATE;

    if(subframe_5)
    {
        len_offset      = timing_struct.symb_starts[0][0];
    }else{
        len_offset      = timing_struct.symb_starts[0][0]+9600;
    }
    prev_offset    += cfo*2*PI*(len_offset+len)/(phy_struct->fs);
    wrap_phase(&prev_offset, 0);
    return len_offset;

}

uint32 LTE_fdd_dl_fs_samp_buf::execute(float* i_data, float* q_data, uint32 len)
{
    Radio* ul_radio = Radio::get_instance();
    switch(recv_state)
    {
        case CELL_SEARCH_STATE:
            cell_search(i_data, q_data, len);
            break;
        case PBCH_DECODE_STATE: 
            ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            pbch_decoding(i_data, q_data, len);    
            if(get_ul_init() == false)
            {
                ul_process();
            }
            break;
        case PDCCH_DECODE_STATE:
            cerr << "Now is PDCCH Encode State, subframe num:"<< current_tti%10 << endl;
            getchar();
            ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            if(current_tti%10 == 5)
            {
                sib1_decoding(i_data, q_data, len);
            }else if(current_tti%10 != 0){
                //clock_t t = clock();
                dlsch_decoding(i_data, q_data, len, current_tti%10);
                //t = clock()-t; fprintf(stderr, "%f secs\n", (float)t/CLOCKS_PER_SEC);
            }else{
                tracking(i_data, q_data, len);
            }
            break;
        case PUCCH_REPORT_STATE:
                if(time_to_op-2 == current_tti)
                {
                    float ul_i_buf[1920];
                    float ul_q_buf[1920];

                    // Uplink Control Channel
                    pucch_encoding(ul_i_buf, ul_q_buf, current_tti%10);

                    // Send to BS
                    ul_radio->send_from_buffer(ul_i_buf, ul_q_buf, 1920);
                }else{
                    ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
                }
                tracking(i_data, q_data, len);
            break;
        case PUSCH_ENCODE_STATE:
            if(time_to_op-2 == current_tti)
            {
                float ul_i_buf[1920];
                float ul_q_buf[1920];

                // Uplink Shared Channel 
                ulsch_encoding(ul_i_buf, ul_q_buf, len);

                ul_radio->send_from_buffer(ul_i_buf, ul_q_buf, 1920);
            }else{
                ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            }    
            tracking(i_data, q_data, len);
            
            break;
        case PHICH_DECODE_STATE:
            ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            if(time_to_op == current_tti)
            {
                phich_decoding(i_data, q_data, len, current_tti%10);
            }else{
                tracking(i_data, q_data, len);
            }
            break;

        case WATING_STATE:
            ul_radio->send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
  
            if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
            {
                if(time_to_op-1 == current_tti)
                    recv_state              = PDCCH_DECODE_STATE;
            }else if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_ULSCH){
                if(time_to_op-3 == current_tti && !ack_or_nack)
                {
                    recv_state              = PUSCH_ENCODE_STATE;
                }else if(ack_or_nack){
                    recv_state              = PDCCH_DECODE_STATE;
                    Alloc_Info.chan_type    = UNDEFINED_CHAN_TYPE;
                }
            }
            
            tracking(i_data, q_data, len);
            break;
        default:
            cerr << "Undefined State..." << endl;
            break;    
    }
}

uint32 LTE_fdd_dl_fs_samp_buf::pbch_decoding(float* i_data, float* q_data, uint32 len=1920)
{   
    LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
    uint32                            N_rb_dl; 
    uint8                             sfn_offset;
    // process one subframe 
    memcpy(&i_buf[0], &i_data[0], len*sizeof(float));
    memcpy(&q_buf[0], &q_data[0], len*sizeof(float));
    subframe->num = 0;

    // compensate cfo first
    // recalculate fcfo
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    cfo += (timing_struct.freq_offset[0]-fcfo);
    fcfo = timing_struct.freq_offset[0];

    freq_shift_Chang(0, len, cfo);
    // transform to freq domain
    liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                      i_buf,
                                      q_buf,
                                      0,
                                      0,
                                      N_id_cell,
                                      4,
                                      subframe);

    // handle rcfo & sco in freq domain and update offset error
    JWLS(phy_struct, i_buf, q_buf, 0, 0, N_id_cell, 1, subframe);

    // next is sib1-decoding, so offset is increased by half frame len
    if_shift_fft_window += sco*1*phy_struct->N_samps_per_subfr;
    prev_offset += cfo*2*PI*(1*phy_struct->N_samps_per_subfr)/(phy_struct->fs);
    time_offset += sco*(1*phy_struct->N_samps_per_subfr);

    wrap_phase(&prev_offset, 0);
    wrap_offset(&time_offset, phy_struct->N_samps_per_symb);

    if(LIBLTE_SUCCESS==liblte_phy_bch_channel_decode(phy_struct,
                                                     subframe,  
                                                     N_id_cell,
                                                     &N_ant,
                                                     rrc_msg.msg,
                                                     &rrc_msg.N_bits,
                                                     &sfn_offset)){
        fprintf(stderr, "liblte_phy_bch_channel_decode succeed...\n");
    }
    else{
        fprintf(stderr, "liblte_phy_bch_channel_decode failed...\n");
        recv_state = CELL_SEARCH_STATE;
        return -1;
    }   
    if(LIBLTE_SUCCESS==liblte_rrc_unpack_bcch_bch_msg(&rrc_msg,
                                                      &mib))
    {
        fprintf(stderr, "liblte_rrc_unpack_bcch_bch_msg succeed...\n");
        switch(mib.dl_bw)
        {
        case LIBLTE_RRC_DL_BANDWIDTH_6:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_1_4MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_15:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_3MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_25:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_5MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_50:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_10MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_75:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_15MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_100:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_20MHZ;
            break;
        }
        liblte_phy_update_n_rb_dl(phy_struct, N_rb_dl);
        sfn       = (mib.sfn_div_4 << 2) + sfn_offset;
        phich_res = liblte_rrc_phich_resource_num[mib.phich_config.res];
        print_mib(&mib);
    }
    else{
        fprintf(stderr, "liblte_rrc_unpack_bcch_bch_msg failed...\n");
    }

    recv_state  = PDCCH_DECODE_STATE;
    current_tti = sfn*10;
    current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);

    return 0;
}   

uint32 LTE_fdd_dl_fs_samp_buf::sib1_decoding(float* i_data, float* q_data, uint32 len=1920)
{
    LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
    uint32                            k;
    uint32                            frame_start_idx = -5*phy_struct->N_samps_per_subfr;
    int32                             subframe_start_idx_offset = 0;

    // initilize data
    memcpy(&i_buf[0], &i_data[0], len*sizeof(float));
    memcpy(&q_buf[0], &q_data[0], len*sizeof(float));
    subframe->num = 5;

    // synchronization
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    cfo += (timing_struct.freq_offset[0]-fcfo);
    fcfo = timing_struct.freq_offset[0];
        
    freq_shift_Chang(0, len, cfo);
    // compensate timing offset in freq domain of one subframe  
    
    while(if_shift_fft_window <-1 || if_shift_fft_window > 1)
    {
        if(if_shift_fft_window <-1){
            if_shift_fft_window = if_shift_fft_window+1;
            subframe_start_idx_offset ++;
        }
        else{
            if_shift_fft_window = if_shift_fft_window-1;
            subframe_start_idx_offset--;
        }
    }
    // transform to freq domain
    liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                      i_buf,
                                      q_buf,
                                      frame_start_idx+subframe_start_idx_offset,
                                      5,
                                      N_id_cell,
                                      4,
                                      subframe);
    // sco compensate
    
    //for(k=0; k<14; k++)  
    //{    
    //    SCO_compensate( phy_struct,
    //                    k,
    //                    0,
    //                    sco,
    //                    subframe);
    //}
    
    // handle rcfo & sco in freq domain and update offset error
    JWLS(phy_struct, i_buf, q_buf, 0, 5, N_id_cell, N_ant, subframe);

    // next is pdcch decoding, so offset is increased by half frame len
    if_shift_fft_window += sco*1*phy_struct->N_samps_per_subfr;
    prev_offset += cfo*2*PI*(1*phy_struct->N_samps_per_subfr)/(phy_struct->fs);
    time_offset += sco*(1*phy_struct->N_samps_per_subfr);

    wrap_phase(&prev_offset, 0);
    wrap_offset(&time_offset, phy_struct->N_samps_per_symb);

    // 
    if(LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                         subframe,
                                                         N_id_cell,
                                                         N_ant,
                                                         phich_res,
                                                         mib.phich_config.dur,
                                                         &pcfich,
                                                         &phich,
                                                         &pdcch))
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode succeed...\n");
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode failed...\n");
    }

    uint32 pdcch_idx = 0;
    for(uint32 i=0; i<pdcch.N_alloc; i++)
        if(pdcch.alloc[i].rnti == LIBLTE_MAC_SI_RNTI)
            pdcch_idx = i;
    
    if(LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                         subframe,
                                                         &pdcch.alloc[pdcch_idx], ///Fixed by Chia-Hao Chang, it has problems... [0] or [2]
                                                         pdcch.N_symbs,
                                                         N_id_cell,
                                                         N_ant,
                                                         rrc_msg.msg,
                                                         &rrc_msg.N_bits))
    {
        fprintf(stderr, "liblte_phy_pdsch_channel_decode succeed...\n");
        //print_msg(&rrc_msg);
        if(LIBLTE_SUCCESS == liblte_rrc_unpack_bcch_dlsch_msg(&rrc_msg,
                                                          &bcch_dlsch_msg))
        {
            if(1 == bcch_dlsch_msg.N_sibs &&
               LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1 == bcch_dlsch_msg.sibs[0].sib_type)
            {
                print_sib1((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&bcch_dlsch_msg.sibs[0].sib);
            }
        }
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdsch_channel_decode failed...\n");
    }
    

    recv_state = PDCCH_DECODE_STATE;
    current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);

    return subframe_start_idx_offset;
}

uint32 LTE_fdd_dl_fs_samp_buf::dlsch_decoding(float* i_data, float* q_data, uint32 len, uint32 subfr_num)
{
    LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
    uint32                            k;
    uint32                            pdcch_idx = 0;
    uint32                            frame_start_idx = -subfr_num*phy_struct->N_samps_per_subfr;
    int32                             subframe_start_idx_offset = 0; 
    
    // initilize data
    memcpy(&i_buf[0], &i_data[0], len*sizeof(float));
    memcpy(&q_buf[0], &q_data[0], len*sizeof(float));
    subframe->num = subfr_num;

    // synchronization
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    cfo += (timing_struct.freq_offset[0]-fcfo);
    fcfo = timing_struct.freq_offset[0];

    freq_shift_Chang(0, len, cfo);
    // compensate timing offset in freq domain of one subframe  
    
    while(if_shift_fft_window <-1 || if_shift_fft_window > 1)
    {
        if(if_shift_fft_window <-1){
            if_shift_fft_window = if_shift_fft_window+1;
            subframe_start_idx_offset++;
        }
        else{
            if_shift_fft_window = if_shift_fft_window-1;
            subframe_start_idx_offset--;
        }
    }

    liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                      i_buf,
                                      q_buf,
                                      frame_start_idx+subframe_start_idx_offset,
                                      subfr_num,
                                      N_id_cell,
                                      4,
                                      subframe);

    // handle rcfo & sco in freq domain and update offset error
    JWLS(phy_struct, i_buf, q_buf, 0, subfr_num, N_id_cell, N_ant, subframe);

    // next is pdcch decoding, so offset is increased by 1 subframe len
    if_shift_fft_window += sco*1*phy_struct->N_samps_per_subfr;
    prev_offset         += cfo*2*PI*(1*phy_struct->N_samps_per_subfr)/(phy_struct->fs);
    time_offset         += sco*(1*phy_struct->N_samps_per_subfr);

    wrap_phase(&prev_offset, 0);
    wrap_offset(&time_offset, phy_struct->N_samps_per_symb);


    if(LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                         subframe,
                                                         N_id_cell,
                                                         N_ant,
                                                         phich_res,
                                                         mib.phich_config.dur,
                                                         &pcfich,
                                                         &phich,
                                                         &pdcch))
    {
        //fprintf(stderr, "liblte_phy_pdcch_channel_decode succeed...\n");
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode failed...\n");
        fprintf(stderr, "=== No control information ===\n");
        current_tti     = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
        sfn             = (current_tti%10==0) ? ++sfn : sfn;
        return -1;
    }

    #pragma omp parallel for
    for(uint32 i=0; i<pdcch.N_alloc; i++)
    {
        if(pdcch.alloc[i].rnti == C_RNTI)
        {
            memcpy(&Alloc_Info, &pdcch.alloc[i], sizeof(LIBLTE_PHY_ALLOCATION_STRUCT));
            
            // New transmission
            if(pdcch.alloc[i].chan_type!=UNDEFINED_CHAN_TYPE
               && pdcch.alloc[i].chan_type==LIBLTE_PHY_CHAN_TYPE_DLSCH)
            {
                if(harq_proc[0].is_toggled(pdcch.alloc[i].ndi))
                {
                    cerr << "New transmission" << endl;
                    harq_proc[0].set_TX_NB(0);
                    harq_proc[0].set_NDI(pdcch.alloc[i].ndi);
                    harq_proc[0].set_RV(0);
                }else{ // Retransmission
                    cerr << "Retransmission" << endl;
                    harq_proc[0].CURRENT_TX_NB++;
                    harq_proc[0].set_RV(pdcch.alloc[i].rv_idx);
                }
            }else if(pdcch.alloc[i].chan_type!=UNDEFINED_CHAN_TYPE
                  && pdcch.alloc[i].chan_type==LIBLTE_PHY_CHAN_TYPE_ULSCH){
                if(ul_harq_proc[0].is_toggled(pdcch.alloc[i].ndi))
                {
                    cerr << "New UL Grant" << endl;
                    ul_harq_proc[0].set_NDI(pdcch.alloc[i].ndi);
                    Alloc_Info.rv_idx = 0;
                    ul_harq_proc[0].set_RV(0);
                    
                }else{ // Retransmission
                    cerr << "UL Retransmission" << endl;
                    ul_harq_proc[0].CURRENT_TX_NB++;
                    if(ul_harq_proc[0].get_RV()==0 || ul_harq_proc[0].get_RV()==1)
                    {
                        Alloc_Info.rv_idx = (ul_harq_proc[0].get_RV()+2);
                    }else{
                        if(ul_harq_proc[0].get_RV()==2)
                            Alloc_Info.rv_idx = 1;
                        else
                            Alloc_Info.rv_idx = 0;
                    }
                    ul_harq_proc[0].set_RV(Alloc_Info.rv_idx);
                }   
            }

            pdcch_idx  = i;
            time_to_op = (current_tti+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1);
        }
    }

    if(Alloc_Info.chan_type == UNDEFINED_CHAN_TYPE)
    {
        fprintf(stderr, "\n=== No control channel at tti %d ===\n\n", current_tti);
        current_tti  = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
        sfn          = (current_tti%10==0) ? ++sfn : sfn;
        return 1;
    }

    if(Alloc_Info.chan_type != UNDEFINED_CHAN_TYPE 
    && Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_ULSCH)
    {
        fprintf(stderr, "\n=== Uplink Grant at tti %d ===\n\n", current_tti);

        // Setup Uplink Message
        set_ul_msg();

        recv_state   = PUSCH_ENCODE_STATE;
        current_tti  = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
        sfn          = (current_tti%10==0) ? ++sfn : sfn;
        return 1;
    }else{
        fprintf(stderr, "\n=== Downlink Scheduling at tti %d ===\n\n", current_tti);
        if(LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                             subframe,
                                                             &pdcch.alloc[pdcch_idx], ///Fixed by Chia-Hao Chang, it has problems... [0] or [2]
                                                             pdcch.N_symbs,
                                                             N_id_cell,
                                                             N_ant,
                                                             rrc_msg.msg,
                                                             &rrc_msg.N_bits))
        {
            //fprintf(stderr, "liblte_phy_pdsch_channel_decode succeed...\n");
            message.push_back(rrc_msg);
            message_idx++;

            print_msg(&rrc_msg);

            // need to report ACK to BS
            harq_proc[0].HARQ_FEEDBACK = 1;

            //recv_state  = PUCCH_REPORT_STATE;

            time_to_op  = (current_tti+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1);
            current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
            sfn         = (current_tti%10==0) ? ++sfn : sfn;
            return 1;
        }
        else
        {
            fprintf(stderr, "liblte_phy_pdsch_channel_decode failed...\n");
            
            // need to report NACK to BS
            harq_proc[0].HARQ_FEEDBACK = 0;

            //recv_state  = PUCCH_REPORT_STATE;

            time_to_op  = (current_tti+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1);
            current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
            sfn         = (current_tti%10==0) ? ++sfn : sfn;
            return -1;
        } 
    }

    current_tti     = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
    sfn             = (current_tti%10==0) ? ++sfn : sfn;

    return 0;
}

void LTE_fdd_dl_fs_samp_buf::ul_process()
{
    liblte_phy_ul_init(phy_struct,
                       N_id_cell,
                       0,      //prach_root_seq_idx,
                       0,      //prach_preamble_format,
                       0,      //prach_zczc,
                       false,  //prach_hs_flag,
                       0,      //group_assignment_pusch,
                       false,  //group_hopping_enabled,
                       false,  //sequence_hopping_enabled,
                       0,      //cyclic_shift,
                       0);     //cyclic_shift_dci)
    ul_init     = true;


    // test
    float samps_re[1920];
    float samps_im[1920];
    uint32 N_det_pre;
    uint32 det_pre;
    uint32 det_ta;
    liblte_phy_generate_prach(phy_struct,
                              5,
                              0, // freq_offset
                              samps_re,
                              samps_im);

    liblte_phy_detect_prach_Chang(phy_struct,
                            samps_re,
                            samps_im,
                            0,
                            &N_det_pre,
                            &det_pre,
                            &det_ta);


}

uint32 LTE_fdd_dl_fs_samp_buf::ulsch_encoding(float* i_data, float* q_data, uint32 subfr_num)
{
    ul_subframe.num = time_to_op%10;
    liblte_phy_pusch_channel_encode(phy_struct,
                                    &Alloc_Info,
                                    N_id_cell,
                                    N_ant,
                                    &ul_subframe);
    liblte_phy_create_ul_subframe(phy_struct,
                                  &ul_subframe,
                                  0, /// ant
                                  i_data,
                                  q_data);

    time_to_op = (time_to_op+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1); // for PHICH decoding
    recv_state = PHICH_DECODE_STATE;
}

uint32  LTE_fdd_dl_fs_samp_buf::tracking(float* i_data, float* q_data, uint32 len)
{
    // one subframe
    // synchronization 
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_data,
                                                           q_data,
                                                           10, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    cfo += (timing_struct.freq_offset[0]-fcfo);
    fcfo = timing_struct.freq_offset[0];

    if_shift_fft_window += sco*1*phy_struct->N_samps_per_subfr;
    prev_offset         += cfo*2*PI*(1*phy_struct->N_samps_per_subfr)/(phy_struct->fs);
    time_offset         += sco*(1*phy_struct->N_samps_per_subfr);

    wrap_phase(&prev_offset, 0);
    wrap_offset(&time_offset, phy_struct->N_samps_per_symb);

    current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
    sfn          = (current_tti%10==0) ? ++sfn : sfn;

    return 0;
}

uint32  LTE_fdd_dl_fs_samp_buf::phich_decoding(float* i_data, float* q_data, uint32 len, uint32 subfr_num)
{
    LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
    uint32                            k;
    uint32                            pdcch_idx = 0;
    uint32                            frame_start_idx = -subfr_num*phy_struct->N_samps_per_subfr;
    int32                             subframe_start_idx_offset = 0; 
    
    // initilize data
    memcpy(&i_buf[0], &i_data[0], len*sizeof(float));
    memcpy(&q_buf[0], &q_data[0], len*sizeof(float));
    subframe->num = subfr_num;

    // synchronization
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    cfo += (timing_struct.freq_offset[0]-fcfo);
    fcfo = timing_struct.freq_offset[0];

    freq_shift_Chang(0, len, cfo);
    // compensate timing offset in freq domain of one subframe  
    
    while(if_shift_fft_window <-1 || if_shift_fft_window > 1)
    {
        if(if_shift_fft_window <-1){
            if_shift_fft_window = if_shift_fft_window+1;
            subframe_start_idx_offset ++;
        }
        else{
            if_shift_fft_window = if_shift_fft_window-1;
            subframe_start_idx_offset--;
        }
    }

    liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                      i_buf,
                                      q_buf,
                                      frame_start_idx+subframe_start_idx_offset,
                                      subfr_num,
                                      N_id_cell,
                                      4,
                                      subframe);

    // handle rcfo & sco in freq domain and update offset error
    JWLS(phy_struct, i_buf, q_buf, 0, subfr_num, N_id_cell, N_ant, subframe);

    // next is pdcch decoding, so offset is increased by 1 subframe len
    if_shift_fft_window += sco*1*phy_struct->N_samps_per_subfr;
    prev_offset         += cfo*2*PI*(1*phy_struct->N_samps_per_subfr)/(phy_struct->fs);
    time_offset         += sco*(1*phy_struct->N_samps_per_subfr);

    wrap_phase(&prev_offset, 0);
    wrap_offset(&time_offset, phy_struct->N_samps_per_symb);


    if(LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                         subframe,
                                                         N_id_cell,
                                                         N_ant,
                                                         phich_res,
                                                         mib.phich_config.dur,
                                                         &pcfich,
                                                         &phich,
                                                         &pdcch))
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode succeed...\n");
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode failed...\n");
        current_tti = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
        sfn          = (current_tti%10==0) ? ++sfn : sfn;
        return -1;
    }

    // PHICH decode
    phich_channel_decode(phy_struct,
                         &Alloc_Info,
                         subframe,
                         N_id_cell,
                         N_ant,
                         LIBLTE_RRC_PHICH_DURATION_NORMAL,
                         &ack_or_nack);  // Uplink HARQ

    if(ack_or_nack)
    {
        fprintf(stderr,"PHICH ACK\n");
        Alloc_Info.rv_idx = 0;

    }else{
        fprintf(stderr,"PHICH NACK\n");    
    }
    
    time_to_op      = (time_to_op+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1);
    recv_state      = WATING_STATE;

    current_tti     = (current_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX+1);
    sfn             = (current_tti%10==0) ? ++sfn : sfn;

    return 0;
}

uint32  LTE_fdd_dl_fs_samp_buf::pucch_encoding(float* i_data, float* q_data, uint32 subfr_num)
{   
    LIBLTE_PHY_SUBFRAME_STRUCT* subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;

    liblte_phy_pucch_formats_1_1a_1b_encode(phy_struct,
                                            (uint8*)&harq_proc[0].HARQ_FEEDBACK, 
                                            1,   // M_bit
                                            N_ant,
                                            1,   // delata_PUCCH_shift
                                            N_id_cell,
                                            0,   // N_slot  
                                            subframe);
    liblte_phy_pucch_formats_1_1a_1b_encode(phy_struct,
                                            (uint8*)&harq_proc[0].HARQ_FEEDBACK, 
                                            1,   // M_bit
                                            N_ant,
                                            1,   // delata_PUCCH_shift
                                            N_id_cell,
                                            1,   // N_slot
                                            subframe);
    liblte_phy_create_ul_subframe(phy_struct,
                                  subframe,
                                  0,
                                  i_data,
                                  q_data);

    time_to_op = (time_to_op+4) % (LTE_FDD_ENB_CURRENT_TTI_MAX+1);

    recv_state = WATING_STATE;

    return 0;
}

void    LTE_fdd_dl_fs_samp_buf::go_next_state()
{
    if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
    {
        recv_state = PDSCH_DECODE_STATE;
    }else if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_ULSCH){
        recv_state = PUSCH_ENCODE_STATE;
    }
}

float LTE_fdd_dl_fs_samp_buf::low_pass_filter(float JWLSE_cfo)
{
    float C1 = (float)1/(float)8;
    float C2 = (float)1/(float)32;
    reg += C2*JWLSE_cfo;
    return (C1*JWLSE_cfo + reg);
}

float moving_avg(float JWLSE_sco)
{
    static vector<float> mv_avg;
    static uint32        count  = 0;
    static uint32        num    = 10;   
    if(count < num){
        count++;
        mv_avg.push_back(JWLSE_sco);
        return ((std::accumulate(mv_avg.begin(), mv_avg.end(), (float)0)/mv_avg.size()));
    }
    else{
        count++;
        mv_avg.push_back(JWLSE_sco);
        mv_avg.erase(mv_avg.begin());
        return ((std::accumulate(mv_avg.begin(), mv_avg.end(), (float)0)/num));
    }
}

void LTE_fdd_dl_fs_samp_buf::SCO_compensate(LIBLTE_PHY_STRUCT          *phy_struct,
                                            uint32                      symbol_offset,
                                            uint32                      start_idx,
                                            float                       sco_tmp,
                                            LIBLTE_PHY_SUBFRAME_STRUCT *subframe)
{
    float   freq_phase   = 0;
    float   N_plus_Ng    = (float)phy_struct->N_samps_per_symb+phy_struct->N_samps_cp_l_else ;
    float   Ng           = (float)phy_struct->N_samps_cp_l_else;
    float   N            = (float)phy_struct->N_samps_per_symb;
    float   comp_re;
    float   comp_im;
    uint32  CP_len;
    uint32  i;
    uint32  j;
    uint32  k;

    float symb_re[128] = {0};
    float symb_im[128] = {0};
    float tmp_re;
    float tmp_im;
    
    for(i=0; i<(phy_struct->FFT_size/2)-phy_struct->FFT_pad_size; i++)
    {
        // Positive spectrum
        symb_re[i+1] = subframe->rx_symb_re[symbol_offset][i+((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)];
        symb_im[i+1] = subframe->rx_symb_im[symbol_offset][i+((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)];

        // Negative spectrum
        symb_re[phy_struct->N_samps_per_symb-i-1] = subframe->rx_symb_re[symbol_offset][((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)-i-1];
        symb_im[phy_struct->N_samps_per_symb-i-1] = subframe->rx_symb_im[symbol_offset][((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)-i-1];
    }
    
    
    for(k=0; k<phy_struct->N_samps_per_symb; k++)
    {
        freq_phase = 2*PI*(k)*((Ng+(N-1)/2)*(sco_tmp)+time_offset)/N;
        comp_re = cosf(-freq_phase);  /// real part
        comp_im = sinf(-freq_phase);  /// imag part
        tmp_re = symb_re[k];
        tmp_im = symb_im[k];
        symb_re[k] = tmp_re*comp_re - tmp_im*comp_im;
        symb_im[k] = tmp_im*comp_re + tmp_re*comp_im;
    }
    //time_offset += N_plus_Ng*sco_tmp;

    for(i=0; i<(phy_struct->FFT_size/2)-phy_struct->FFT_pad_size; i++)
    {
        // Positive spectrum
        subframe->rx_symb_re[symbol_offset][i+((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)] = symb_re[i+1];
        subframe->rx_symb_im[symbol_offset][i+((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)] = symb_im[i+1];

        // Negative spectrum
        subframe->rx_symb_re[symbol_offset][((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)-i-1] = symb_re[phy_struct->N_samps_per_symb-i-1];
        subframe->rx_symb_im[symbol_offset][((phy_struct->FFT_size/2)-phy_struct->FFT_pad_size)-i-1] = symb_im[phy_struct->N_samps_per_symb-i-1];
    }    
}

LIBLTE_ERROR_ENUM LTE_fdd_dl_fs_samp_buf::JWLS(LIBLTE_PHY_STRUCT          *phy_struct,
                                               float                      *i_samps,
                                               float                      *q_samps,
                                               uint32                      frame_start_idx,
                                               uint8                       subfr_num,
                                               uint32                      N_id_cell,
                                               uint8                       N_ant,
                                               LIBLTE_PHY_SUBFRAME_STRUCT *subframe)
{
    LIBLTE_ERROR_ENUM  err = LIBLTE_ERROR_INVALID_INPUTS;
    float              *sym_re_1                                                                   ;
    float              *sym_im_1                                                                   ;
    float              *sym_re_2                                                                   ;
    float              *sym_im_2                                                                   ;
    float              Epsilon          =   0                                                      ;
    float              Delta            =   0                                                      ;
    float              tmp_re           =   0                                                      ;
    float              tmp_im           =   0                                                      ;
    float              theta            =   0                                                      ;
    uint32             v_shift          = N_id_cell % 6                                            ;
    uint32             subfr_start_idx  = frame_start_idx + subfr_num*phy_struct->N_samps_per_subfr;
    uint32             N_sym;
    uint32             m_prime;
    uint32             i;
    uint32             j;
    uint32             k = 0;
    uint32             p;
    uint32             v[5];
    uint32             sym[5];
    uint32             z;

    liblte_phy_map_crs(phy_struct,
                       subframe,
                       N_id_cell,
                       N_ant);

    for(p=0; p<N_ant; p++)
    {
        // Define v, sym, and N_sym
        if(p == 0)
        {
            v[0]   = 0;
            v[1]   = 3;
            v[2]   = 0;
            v[3]   = 3;
            v[4]   = 0;
            sym[0] = 0;
            sym[1] = 4;
            sym[2] = 7;
            sym[3] = 11;
            sym[4] = 14;
            N_sym  = 5;
        }else if(p == 1){
            v[0]   = 3;
            v[1]   = 0;
            v[2]   = 3;
            v[3]   = 0;
            v[4]   = 3;
            sym[0] = 0;
            sym[1] = 4;
            sym[2] = 7;
            sym[3] = 11;
            sym[4] = 14;
            N_sym  = 5;
        }else if(p == 2){
            v[0]   = 0;
            v[1]   = 3;
            v[2]   = 0;
            sym[0] = 1;
            sym[1] = 8;
            sym[2] = 15;
            N_sym  = 3;
        }else{ // p == 3
            v[0]   = 3;
            v[1]   = 6;
            v[2]   = 3;
            sym[0] = 1;
            sym[1] = 8;
            sym[2] = 15;
            N_sym  = 3;
        }

        //fprintf(stderr, "Start JWLS...\n");

        sym_re_1 = &subframe->rx_ce_re[p][sym[0]][0];
        sym_im_1 = &subframe->rx_ce_im[p][sym[0]][0];
        sym_re_2 = &subframe->rx_ce_re[p][sym[0]+1][0];
        sym_im_2 = &subframe->rx_ce_im[p][sym[0]+1][0];

        float wk            = 0;
        float wk_yk         = 0;
        float N_plus_Ng     = (float)phy_struct->N_samps_per_symb+phy_struct->N_samps_cp_l_else ;
        float N             = (float)phy_struct->N_samps_per_symb;
        float sum_theta     = 0;
        
        #pragma omp parallel for
        for(i=0; i<2*phy_struct->N_rb_dl; i++)
        {
            k           = 6*i + (v[0] + v_shift)%6;   /// k == alpha
            tmp_re      = sym_re_2[k]*sym_re_1[k] + sym_im_2[k]*sym_im_1[k];
            tmp_im      = sym_im_2[k]*sym_re_1[k] - sym_re_2[k]*sym_im_1[k];
            theta       = atan2f(tmp_im, tmp_re);
            sum_theta  += theta;
        }
        Epsilon         = sum_theta/(2*PI*(1*N_plus_Ng)*12/N);
        prev_rcfo       = residual_cfo;
        residual_cfo    = low_pass_filter(Epsilon)*15000;
        cfo            += (residual_cfo-prev_rcfo);
        /*{
        for(i=0; i<2*phy_struct->N_rb_dl; i++)
        {
            k           = 6*i + (v[0] + v_shift)%6;   /// k == alpha
            tmp_re      = sym_re_2[k]*sym_re_1[k] + sym_im_2[k]*sym_im_1[k];
            tmp_im      = sym_im_2[k]*sym_re_1[k] - sym_re_2[k]*sym_im_1[k];
            theta       = atan2f(tmp_im, tmp_re);
            wk         += sqrt(sym_re_1[k]*sym_re_1[k]+sym_im_1[k]*sym_im_1[k])*sqrt(sym_re_2[k]*sym_re_2[k]+sym_im_2[k]*sym_im_2[k]);
            wk_yk      += theta*sqrt(sym_re_1[k]*sym_re_1[k]+sym_im_1[k]*sym_im_1[k])*sqrt(sym_re_2[k]*sym_re_2[k]+sym_im_2[k]*sym_im_2[k]);
        }
        
        Epsilon = (wk_yk*15000)/(wk*2*PI*(1*N_plus_Ng)/phy_struct->N_samps_per_symb);
        fprintf(stderr, "!!!!!!!!!!!!!!!!!!!!!! rcfo %f\n", Epsilon);
        residual_cfo = low_pass_filter(Epsilon);
        //fprintf(stderr, "!!!!!!!!!!!!!!!!!!!!!! rcfo %f\n", residual_cfo);
        }*/

        /********* SCO Estimation *********/
        float theta_alpha   = 0;
        float alpha_2       = 0;
        
        #pragma omp parallel for
        for(i=0; i<2*phy_struct->N_rb_dl; i++)
        {
            k            = 6*i + (v[0] + v_shift)%6;
            tmp_re       = sym_re_2[k]*sym_re_1[k] + sym_im_2[k]*sym_im_1[k];
            tmp_im       = sym_im_2[k]*sym_re_1[k] - sym_re_2[k]*sym_im_1[k];
            theta        = atan2f(tmp_im, tmp_re);
            if(k >= 36)
            {
                theta_alpha += theta*((k+92+1)%128);
                alpha_2     += ((k+92+1)%128)*((k+92+1)%128);
            }else{
                theta_alpha += theta*((k+92)%128);
                alpha_2     += ((k+92)%128)*((k+92)%128);
            }
        }
        Delta = (theta_alpha) / ((2*PI*N_plus_Ng/N)*(alpha_2));
        Delta = moving_avg(Delta);
        sco = Delta;

        /*{
        float w_alpha_square= 0;
        float w_theta_alpha = 0;
        float w_alpha_2     = 0;
        float w_theta       = 0;
        float w_alpha       = 0;
        float w             = 0;
        float channel_gain  = 0;
        
        float alpha_2       = 0;
        float phase_sum     = 0;
        for(i=0; i<12*phy_struct->N_rb_dl-1; i++)
        {
            k = i;//6*i + (v[0] + v_shift)%6;   /// k == alpha
            for(j=0; j<6; j++){
                tmp_re       = sym_re_2[k+j]*sym_re_1[k+j] + sym_im_2[k+j]*sym_im_1[k+j];
                tmp_im       = sym_im_2[k+j]*sym_re_1[k+j] - sym_re_2[k+j]*sym_im_1[k+j];
                theta        += atan2f(tmp_im, tmp_re);
            }
            theta /= 6;

            //channel_gain = sqrt(sym_re_1[k]*sym_re_1[k]+sym_im_1[k]*sym_im_1[k])*sqrt(sym_re_2[k]*sym_re_2[k]+sym_im_2[k]*sym_im_2[k]);
            channel_gain = sym_re_1[k]*sym_re_1[k]+sym_im_1[k]*sym_im_1[k];
            w           += channel_gain;
            w_theta     += theta*channel_gain;
            w_alpha     += k*channel_gain;
            w_alpha_2   += (pow(k,2)*channel_gain);
            w_theta_alpha += (theta*k*channel_gain);

            theta_alpha += theta*k;
            alpha_2     += pow(k,2);
        }  
        w_alpha_square = pow(w_alpha,2);
        Delta = (w*w_theta_alpha - w_alpha*w_theta) / ((2*PI*N_plus_Ng/N)*(w*w_alpha_2-w_alpha_square));
        Delta = moving_avg(Delta);
        fprintf(stderr, "SCO is %f\n", Delta);
        sco = Delta;
        }*/
        err = LIBLTE_SUCCESS;
    }
    return (err);
}

void LTE_fdd_dl_fs_samp_buf::find_half_frame_boundaires(LIBLTE_PHY_STRUCT               *phy_struct,
                                                        LIBLTE_PHY_COARSE_TIMING_STRUCT *boundary,
                                                        uint32                           times)
{
    uint32 i;
    uint32 j;
    uint32 off_len = 0;
    for(i=0; i<times; i++)
    {        
        liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                               &i_buf[off_len],
                                                               &q_buf[off_len],
                                                               20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                               &boundary[i]);
        fprintf(stderr, "coarse timing start is %d...\n",off_len+boundary[i].symb_starts[0][0]);
        for(j=0; j<7; j++)
        {
            boundary[i].symb_starts[0][j] += off_len;
        }
        fprintf(stderr, "coarse timing start is %d...\n",boundary[i].symb_starts[0][0]);
        off_len += 137;
    }
    //getchar();
}

int LTE_fdd_dl_fs_samp_buf::handle_dl(float* i_recv_buf, float* q_recv_buf, int len)
{
    LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT[100];
    LIBLTE_PHY_COARSE_TIMING_STRUCT   boundary[70];
    LIBLTE_PHY_PCFICH_STRUCT          pcfich;
    LIBLTE_PHY_PHICH_STRUCT           phich;
    LIBLTE_PHY_PDCCH_STRUCT           pdcch;
    float                             pss_thresh;
    float                             freq_offset;
    uint8                             sfn_offset;
    int32                             done_flag         = 0;
    uint32                            i;
    uint32                            k;
    uint32                            pss_symb;
    uint32                            frame_start_idx;
    uint32                            num_samps_needed  = 0;
    uint32                            samps_to_copy;
    uint32                            N_rb_dl;
    uint32                            subframe_idx      = 0;                
    int32                             offset_len        = 0;
    size_t                            line_size         = LINE_MAX;
    ssize_t                           N_line_chars;
    char                             *line;
    bool                              process_samples   = false;
    bool                              copy_input        = false;
    bool                              done              = false;
    bool                              subframe_5        = false;
    

    liblte_phy_init(&phy_struct,
                    fs,
                    LIBLTE_PHY_INIT_N_ID_CELL_UNKNOWN,
                    1,
                    LIBLTE_PHY_N_RB_DL_1_4MHZ,
                    LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
                    liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1]);
   
    memcpy(&i_buf[residual_len], &i_recv_buf[0], len*sizeof(float));
    memcpy(&q_buf[residual_len], &q_recv_buf[0], len*sizeof(float));

    /************Find half frame length symbol boundary***********/
    
    find_half_frame_boundaires(phy_struct, boundary, 70);
    memcpy(&timing_struct, &boundary[0], sizeof(LIBLTE_PHY_COARSE_TIMING_STRUCT));

    /************************************************************/
    
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           i_buf,
                                                           q_buf,
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    
    fprintf(stderr, "coarse timing start is %d...\n",timing_struct.symb_starts[0][0]);
    fprintf(stderr, "fractional freq offset is %10f...\n",timing_struct.freq_offset[0]);
    cfo += timing_struct.freq_offset[0];
    freq_shift_Chang(residual_len, phy_struct->N_samps_per_frame, timing_struct.freq_offset[0]);

    
    /*********************************************************************************************************/

    liblte_phy_find_pss_and_fine_timing_Chang_v2(phy_struct,
                                              &i_buf[offset_len],
                                              &q_buf[offset_len],
                                              boundary,
                                              timing_struct.symb_starts[0],
                                              &N_id_2,
                                              &pss_symb,
                                              &pss_thresh,
                                              &freq_offset);

    cfo += freq_offset;
    freq_shift_Chang(residual_len, phy_struct->N_samps_per_frame, freq_offset);  ///ICFO
    
    timing_struct.freq_offset[0] += freq_offset;
    fprintf(stderr, "Timing start index is %d...\n",timing_struct.symb_starts[0][0]);
    fprintf(stderr, "Compensate ICFO by PSS and freq offset is %10f...\n",freq_offset);
    fprintf(stderr, "N_id_2 is %d...\n",N_id_2);


    liblte_phy_find_sss_Chang(phy_struct,
                              &i_buf[offset_len],
                              &q_buf[offset_len],
                              N_id_2,
                              timing_struct.symb_starts[0],
                              pss_thresh,
                              &N_id_1,
                              &frame_start_idx,
                              subframe_5);
    fprintf(stderr, "\nN_ID_1:%d and N_ID_2:%d...\nframe_start_idx : %d\n", N_id_1, N_id_2, frame_start_idx);
    if(subframe_5)
    {
        fprintf(stderr, "start from subframe 5\n");
        for(i=0; i<7; i++)
            timing_struct.symb_starts[0][i] += (9600+9600*0.000);
        frame_start_idx += (9600+9600*0.00);
        subframe_idx = 0;
    }
    else
    {
        fprintf(stderr, "start from subframe 0\n");
        subframe_idx = 0;
    }
    N_id_cell = N_id_1*3+N_id_2;

    //getchar();


    /*********************************************************************************************************/
    uint32 subframe_start_idx_offset = 0;
    uint32 subframe_start_idx = frame_start_idx;
    float  prev_phase_for_rcfo = 0;  /// for_cfo
    float  avg_sco = 0;
    uint32 num_subframe = (len-frame_start_idx)/(phy_struct->N_samps_per_subfr);



    for(i=0; i</*num_subframe*/10; i++)
    {
        
        freq_shift_rcfo(subframe_start_idx + residual_len, /// may have bugs...  
                        phy_struct->N_samps_per_subfr,
                        &prev_phase_for_rcfo,
                        residual_cfo);
        
        /***/
        int start = 0;
        for(k=0; k<14; k++)  
        {    
            SCO_compensate( phy_struct,
                            k,
                            start,
                            sco,
                            &subframe[(i+subframe_idx)]);
        }  
        /**************/

        
        while(if_shift_fft_window <-1 || if_shift_fft_window > 1)
        {
            if(if_shift_fft_window <-1){
                if_shift_fft_window = if_shift_fft_window+1;
                subframe_start_idx_offset ++;
                subframe_start_idx ++;
            }
            else{
                if_shift_fft_window = if_shift_fft_window-1;
                subframe_start_idx_offset--;
                subframe_start_idx --;
            }
        }
        subframe[i].num = i;
        liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                          i_buf,
                                          q_buf,
                                          frame_start_idx + subframe_start_idx_offset,
                                          (i+subframe_idx)%10,
                                          N_id_cell,
                                          4,
                                          &subframe[(i+subframe_idx)]);
        
        fprintf(stderr, "Each subframe idx %d\n", frame_start_idx + subframe_start_idx_offset +i*phy_struct->N_samps_per_subfr);
        JWLS(phy_struct, i_buf, q_buf, frame_start_idx, 0, N_id_cell, 1, &subframe[(i+subframe_idx)]);
        
        avg_sco             += sco;
        //sco                  = -0.000000; /////  Fixed by me, observing from USRP 2932... 
        if_shift_fft_window += sco*phy_struct->N_samps_per_subfr;
        
        if(i==0)
            time_offset += sco*frame_start_idx;

       
        fprintf(stderr, "SCO is %f...\n", sco);
        fprintf(stderr, "residual_cfo is %f...\n", residual_cfo);

        subframe_start_idx += phy_struct->N_samps_per_subfr; 
    }
    avg_sco /= 10;  fprintf(stderr, "average sco is %f...\n", avg_sco);

    prev_offset    +=   prev_phase_for_rcfo + cfo*2*PI*len/(phy_struct->fs);
    len_offset      =   frame_start_idx+subframe_start_idx_offset+10*phy_struct->N_samps_per_subfr;
    residual_len    =   len - len_offset;

    memset(&i_buf[0], 0, len*sizeof(float));
    memset(&q_buf[0], 0, len*sizeof(float));
    memcpy(&i_buf[0], &i_recv_buf[len_offset], residual_len*sizeof(float));
    memcpy(&q_buf[0], &q_recv_buf[len_offset], residual_len*sizeof(float));


    if(LIBLTE_SUCCESS==liblte_phy_bch_channel_decode(phy_struct,
                                                     &subframe[subframe_idx],  /// dependent on PSS timing 0 or 5
                                                     N_id_cell,
                                                     &N_ant,
                                                     rrc_msg.msg,
                                                     &rrc_msg.N_bits,
                                                     &sfn_offset)){
        fprintf(stderr, "liblte_phy_bch_channel_decode succeed...\n");
        LTE_File record("PBCH_Equalized_IQ.dat");
        record.record_OFDM(phy_struct->bch_x_re, phy_struct->bch_x_im, 240);
    }
    else{
        fprintf(stderr, "liblte_phy_bch_channel_decode failed...\n");
    }   
    if(LIBLTE_SUCCESS==liblte_rrc_unpack_bcch_bch_msg(&rrc_msg,
                                                      &mib)){
        fprintf(stderr, "liblte_rrc_unpack_bcch_bch_msg succeed...\n");
        switch(mib.dl_bw)
        {
        case LIBLTE_RRC_DL_BANDWIDTH_6:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_1_4MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_15:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_3MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_25:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_5MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_50:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_10MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_75:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_15MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_100:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_20MHZ;
            break;
        }
        liblte_phy_update_n_rb_dl(phy_struct, N_rb_dl);
        sfn       = (mib.sfn_div_4 << 2) + sfn_offset;
        phich_res = liblte_rrc_phich_resource_num[mib.phich_config.res];
        print_mib(&mib);
    }
    else{
        fprintf(stderr, "liblte_rrc_unpack_bcch_bch_msg failed...\n");
    }
    
    if(LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                         &subframe[3],
                                                         N_id_cell,
                                                         N_ant,
                                                         phich_res,
                                                         mib.phich_config.dur,
                                                         &pcfich,
                                                         &phich,
                                                         &pdcch))
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode succeed...\n");
        LTE_File record("PDCCH_Equalized_IQ.dat");
        record.record_OFDM(phy_struct->pdcch_x_re, phy_struct->pdcch_x_im, 144);
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdcch_channel_decode failed...\n");
        LTE_File record("PDCCH_Equalized_IQ_failed.dat");
        record.record_OFDM(phy_struct->pdcch_x_re, phy_struct->pdcch_x_im, 144);
    }

    uint32 pdcch_idx = 0;
    for(i=0; i<pdcch.N_alloc; i++)
    {
        if(pdcch.alloc[i].rnti == LIBLTE_MAC_C_RNTI_START)
            pdcch_idx = i;
    }
    if(LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                         &subframe[3],
                                                         &pdcch.alloc[pdcch_idx], ///Fixed by Chia-Hao Chang, it has problems... [0] or [2]
                                                         pdcch.N_symbs,
                                                         N_id_cell,
                                                         N_ant,
                                                         rrc_msg.msg,
                                                         &rrc_msg.N_bits))
    {
        fprintf(stderr, "liblte_phy_pdsch_channel_decode succeed...\n");
        print_msg(&rrc_msg);

        LTE_File record("PDSCH_Equalized_IQ.dat");
        record.record_OFDM(phy_struct->pdsch_x_re, phy_struct->pdsch_x_im, 240);
    }
    else
    {
        fprintf(stderr, "liblte_phy_pdsch_channel_decode failed...\n");
    } 

    return residual_len;
}

int32 LTE_fdd_dl_fs_samp_buf::work(LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ frame)
{
    LIBLTE_PHY_SUBFRAME_STRUCT  subframe;
    LIBLTE_PHY_PCFICH_STRUCT    pcfich;
    LIBLTE_PHY_PHICH_STRUCT     phich;
    LIBLTE_PHY_PDCCH_STRUCT     pdcch;
    float                       pss_thresh;
    float                       freq_offset;
    int32                       done_flag = 0;
    uint32                      i;
    uint32                      pss_symb;
    uint32                      frame_start_idx;
    uint32                      num_samps_needed = 0;
    uint32                      samps_to_copy;
    uint32                      N_rb_dl;
    size_t                      line_size = LINE_MAX;
    ssize_t                     N_line_chars;
    char                       *line;
    uint8                       sfn_offset;
    bool                        process_samples = false;
    bool                        copy_input      = false;
    bool                        done            = false;

    /*Iniatialize physical struct*/
    liblte_phy_init(&phy_struct,
                    fs,
                    LIBLTE_PHY_INIT_N_ID_CELL_UNKNOWN,
                    1,
                    LIBLTE_PHY_N_RB_DL_1_4MHZ,
                    LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
                    liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1]);
   
    num_samps_needed = phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;

    /* Test by adding CFO, it would be useless*/
    freq_offset = 0;
    float  f_samp_re;
    float  f_samp_im;
    float  tmp_i;
    float  tmp_q;

    for(i=0; i<(LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ); i++)
    {
        f_samp_re = cosf((i+1)*(freq_offset)*2*M_PI/phy_struct->fs);
        f_samp_im = sinf((i+1)*(freq_offset)*2*M_PI/phy_struct->fs);
        tmp_i     = frame.i_buf[0][i];
        tmp_q     = frame.q_buf[0][i];
        frame.i_buf[0][i]  = tmp_i*f_samp_re - tmp_q*f_samp_im;
        frame.q_buf[0][i]  = tmp_q*f_samp_re + tmp_i*f_samp_im;
    }
    //SCO(i_buf, frame.i_buf[0], 1000, 0, (LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ));
    //SCO(q_buf, frame.q_buf[0], 1000, 0, (LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ));

    memcpy(i_buf, frame.i_buf[0], LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*sizeof(float));
    memcpy(q_buf, frame.q_buf[0], LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*sizeof(float));
    fprintf(stderr, "add CFO & SCO ...\n");
    //getchar();
    /*SISO case*/
    /*******************************************/
    while(!done)
    {
        switch(state)
        {
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH:
            fprintf(stderr, "Next state is Coarse timing search...\n");
            //getchar();
            if(LIBLTE_SUCCESS == liblte_phy_dl_find_coarse_timing_and_freq_offset(phy_struct,
                                                                                  i_buf,
                                                                                  q_buf,
                                                                                  20/*COARSE_TIMING_N_SLOTS*/,
                                                                                  &timing_struct))
            {
                if(corr_peak_idx < timing_struct.n_corr_peaks)
                {
                    // Correct frequency error
                    freq_shift(timing_struct.symb_starts[0][0], LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ, timing_struct.freq_offset[corr_peak_idx]);
                    // Search for PSS and fine timing
                    state            = LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH;
                    num_samps_needed = phy_struct->N_samps_per_subfr * PSS_AND_FINE_TIMING_SEARCH_NUM_SUBFRAMES;
                    
                    fprintf(stderr, "Timing start index is %d...\n",timing_struct.symb_starts[0][0]);
                    fprintf(stderr, "Compensate FCFO by CP and freq offset is %10f...\n",timing_struct.freq_offset[corr_peak_idx]);
                    getchar();

                    /*
                    liblte_phy_dl_find_coarse_timing_and_freq_offset(phy_struct,
                                                                     i_buf,
                                                                     q_buf,
                                                                     20,
                                                                     &timing_struct);
                    fprintf(stderr, "Timing start index is %d...\n",timing_struct.symb_starts[0][0]);
                    fprintf(stderr, "Compensate FCFO by CP and freq offset is %10f...\n",timing_struct.freq_offset[corr_peak_idx]);
                    getchar();
                    freq_shift(timing_struct.symb_starts[0][0], LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ, timing_struct.freq_offset[corr_peak_idx]);

                    liblte_phy_dl_find_coarse_timing_and_freq_offset(phy_struct,
                                                                     i_buf,
                                                                     q_buf,
                                                                     20,
                                                                     &timing_struct);
                    fprintf(stderr, "Timing start index is %d...\n",timing_struct.symb_starts[0][0]);
                    fprintf(stderr, "Compensate FCFO by CP and freq offset is %10f...\n",timing_struct.freq_offset[corr_peak_idx]);
                    getchar();
                    freq_shift(timing_struct.symb_starts[0][0], LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ, timing_struct.freq_offset[corr_peak_idx]);
                    */

                }else{
                    // No more peaks, so signal that we are done
                    done_flag = -1;
                }
            }else{
                // Stay in coarse timing search
                samp_buf_r_idx   += phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
                num_samps_needed  = phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
            }
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH:
            fprintf(stderr, "Next state is PSS search...\n");
            //getchar();
            if(LIBLTE_SUCCESS == liblte_phy_find_pss_and_fine_timing(phy_struct,
                                                                           i_buf,
                                                                           q_buf,
                                                                           timing_struct.symb_starts[corr_peak_idx],
                                                                           &N_id_2,
                                                                           &pss_symb,
                                                                           &pss_thresh,
                                                                           &freq_offset))
            {
                if(fabs(freq_offset) > 100)
                {
                    freq_shift(0, LIBLTE_PHY_N_SAMPS_PER_FRAME_1_92MHZ*8, freq_offset);
                    timing_struct.freq_offset[corr_peak_idx] += freq_offset;
                    fprintf(stderr, "Compensate ICFO by PSS and freq offset is %10f...\n",freq_offset);
                    //getchar();
                }
                // Search for SSS
                state            = LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH;
                num_samps_needed = phy_struct->N_samps_per_subfr * SSS_SEARCH_NUM_SUBFRAMES;
                
                /* Debug : Check whether CFO is compensated... 
                for(int index=0; index<100; index++){
                    cerr<<frame.i_buf[0][index]<<endl;
                    cerr<<i_buf[index]<<endl;
                }
                */
    
                fprintf(stderr, "Next state is SSS search...\n");
                //getchar();
    
    
            }else{
                // Go back to coarse timing search
                state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
                samp_buf_r_idx   += phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
                num_samps_needed  = phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
            }
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH:
            if(LIBLTE_SUCCESS == liblte_phy_find_sss(phy_struct,
                                                     i_buf,
                                                     q_buf,
                                                     N_id_2,
                                                     timing_struct.symb_starts[corr_peak_idx],
                                                     pss_thresh,
                                                     &N_id_1,
                                                     &frame_start_idx))
            {
                N_id_cell = 3*N_id_1 + N_id_2;
                for(i=0; i<N_decoded_chans; i++)
                {
                    if(N_id_cell == decoded_chans[i])
                    {
                        break;
                    }
                }
                if(i != N_decoded_chans)
                {
                    // Go back to coarse timing search
                    state = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
                    corr_peak_idx++;
                    init();
                }else{
                    // Decode BCH
                    state = LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE;
                    while(frame_start_idx < samp_buf_r_idx)
                    {
                        frame_start_idx += phy_struct->N_samps_per_frame;
                    }
                    samp_buf_r_idx   = frame_start_idx;
                    num_samps_needed = phy_struct->N_samps_per_frame * BCH_DECODE_NUM_FRAMES;

                    fprintf(stderr, "\nN_ID_1:%d and N_ID_2:%d...\nframe_start_idx : %d ", N_id_1, N_id_2, frame_start_idx);
                    //getchar();
                }
            }else{
                // Go back to coarse timing search
                state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
                samp_buf_r_idx   += phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
                num_samps_needed  = phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
            }
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE:
            fprintf(stderr, "Next state is Broadcast Channel decoding...\n");


            if(LIBLTE_SUCCESS == liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                                                   i_buf,
                                                                   q_buf,
                                                                   samp_buf_r_idx,
                                                                   0,
                                                                   N_id_cell,
                                                                   1,
                                                                   &subframe) &&
               LIBLTE_SUCCESS == liblte_phy_bch_channel_decode(phy_struct,
                                                               &subframe,
                                                               N_id_cell,
                                                               &N_ant,
                                                               rrc_msg.msg,
                                                               &rrc_msg.N_bits,
                                                               &sfn_offset) &&
               LIBLTE_SUCCESS == liblte_rrc_unpack_bcch_bch_msg(&rrc_msg,
                                                                &mib))
            {
                fprintf(stderr, "liblte_phy_bch_channel_decode...\nliblte_phy_get_dl_subframe_and_ce...\nliblte_rrc_unpack_bcch_bch_msg succeed...\n");
                //getchar();

                switch(mib.dl_bw)
                {
                case LIBLTE_RRC_DL_BANDWIDTH_6:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_1_4MHZ;
                    break;
                case LIBLTE_RRC_DL_BANDWIDTH_15:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_3MHZ;
                    break;
                case LIBLTE_RRC_DL_BANDWIDTH_25:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_5MHZ;
                    break;
                case LIBLTE_RRC_DL_BANDWIDTH_50:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_10MHZ;
                    break;
                case LIBLTE_RRC_DL_BANDWIDTH_75:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_15MHZ;
                    break;
                case LIBLTE_RRC_DL_BANDWIDTH_100:
                    N_rb_dl = LIBLTE_PHY_N_RB_DL_20MHZ;
                    break;
                }
                liblte_phy_update_n_rb_dl(phy_struct, N_rb_dl);
                sfn       = (mib.sfn_div_4 << 2) + sfn_offset;
                phich_res = liblte_rrc_phich_resource_num[mib.phich_config.res];
                print_mib(&mib);
                // Add this channel to the list of decoded channels
                decoded_chans[N_decoded_chans++] = N_id_cell;
                if(LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX == N_decoded_chans)
                {
                    done_flag = -1;
                }

                // Decode PDSCH for SIB1
                state = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1;
                if((sfn % 2) != 0)
                {
                    //samp_buf_r_idx += phy_struct->N_samps_per_frame;
                    //sfn++;
                    samp_buf_r_idx += 0;
                    
                }
                num_samps_needed = phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;
            }else{
                // Go back to coarse timing search
                fprintf(stderr, "BCH decode failed...\n");
                getchar();
                state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
                samp_buf_r_idx   += phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
                num_samps_needed  = phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
            }
            break;   
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1:
//#ifdef DEBUG        
            if(LIBLTE_SUCCESS == liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                                                   i_buf,
                                                                   q_buf,
                                                                   samp_buf_r_idx,
                                                                   2,
                                                                   N_id_cell,
                                                                   N_ant,
                                                                   &subframe))
            {
                fprintf(stderr, "PDSCH liblte_phy_get_dl_subframe_and_ce succeed...\n");
            }
            else
            {
                fprintf(stderr, "PDSCH liblte_phy_get_dl_subframe_and_ce failed...\n");
            }
            getchar();
            
            
            //LTE_File Record_pdcch("pdcch_succ.dat");
    
            

            if(LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 phich_res,
                                                                 mib.phich_config.dur,
                                                                 &pcfich,
                                                                 &phich,
                                                                 &pdcch))
            {
                fprintf(stderr, "liblte_phy_pdcch_channel_decode succeed...\n");
            }
            else
            {
                fprintf(stderr, "liblte_phy_pdcch_channel_decode failed...\n");
            }
            getchar();
            //pdcch.alloc[2].tbs = 16; ///need to fix !!!
            if(LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 &pdcch.alloc[0], ///Fixed by Chia-Hao Chang
                                                                 pdcch.N_symbs,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 rrc_msg.msg,
                                                                 &rrc_msg.N_bits))
            {
                fprintf(stderr, "liblte_phy_pdsch_channel_decode succeed...\n");
            }
            else
            {
                fprintf(stderr, "liblte_phy_pdsch_channel_decode failed...\n");
            }
            getchar();
        
//#else

//#endif
            if(LIBLTE_SUCCESS == liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                                                   i_buf,
                                                                   q_buf,
                                                                   samp_buf_r_idx,
                                                                   5,
                                                                   N_id_cell,
                                                                   N_ant,
                                                                   &subframe) &&
               LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 phich_res,
                                                                 mib.phich_config.dur,
                                                                 &pcfich,
                                                                 &phich,
                                                                 &pdcch) &&
               LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 &pdcch.alloc[0],
                                                                 pdcch.N_symbs,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 rrc_msg.msg,
                                                                 &rrc_msg.N_bits) &&
               LIBLTE_SUCCESS == liblte_rrc_unpack_bcch_dlsch_msg(&rrc_msg,
                                                                  &bcch_dlsch_msg))
            {
                fprintf(stderr, "liblte_phy_pdcch_channel_decode...\nliblte_phy_pdsch_channel_decode...\nliblte_rrc_unpack_bcch_dlsch_msg succeed...\n");

                if(1                                == bcch_dlsch_msg.N_sibs &&
                   LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1 == bcch_dlsch_msg.sibs[0].sib_type)
                {
                    print_sib1((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&bcch_dlsch_msg.sibs[0].sib);
                }
                // Decode all PDSCHs
                state            = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC;
                N_sfr            = 0;
                num_samps_needed = phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
            }else{
                // Try to decode SIB1 again
                samp_buf_r_idx   += phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;
                sfn              += 2;
                num_samps_needed  = phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;
            }
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC:
            if(LIBLTE_SUCCESS == liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                                                   i_buf,
                                                                   q_buf,
                                                                   samp_buf_r_idx,
                                                                   N_sfr,
                                                                   N_id_cell,
                                                                   N_ant,
                                                                   &subframe) &&
               LIBLTE_SUCCESS == liblte_phy_pdcch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 phich_res,
                                                                 mib.phich_config.dur,
                                                                 &pcfich,
                                                                 &phich,
                                                                 &pdcch) &&
               LIBLTE_SUCCESS == liblte_phy_pdsch_channel_decode(phy_struct,
                                                                 &subframe,
                                                                 &pdcch.alloc[0],
                                                                 pdcch.N_symbs,
                                                                 N_id_cell,
                                                                 N_ant,
                                                                 rrc_msg.msg,
                                                                 &rrc_msg.N_bits))
            {
                if(LIBLTE_MAC_SI_RNTI == pdcch.alloc[0].rnti &&
                   LIBLTE_SUCCESS     == liblte_rrc_unpack_bcch_dlsch_msg(&rrc_msg,
                                                                          &bcch_dlsch_msg))
                {
                    for(i=0; i<bcch_dlsch_msg.N_sibs; i++)
                    {
                        switch(bcch_dlsch_msg.sibs[i].sib_type)
                        {
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1:
                            print_sib1((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2:
                            print_sib2((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3:
                            print_sib3((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4:
                            print_sib4((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5:
                            print_sib5((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6:
                            print_sib6((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7:
                            print_sib7((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8:
                            print_sib8((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *)&bcch_dlsch_msg.sibs[i].sib);
                            break;
                        default:
                            printf("Not handling SIB %u\n", bcch_dlsch_msg.sibs[i].sib_type);
                            break;
                        }
                    }
                }else if(LIBLTE_MAC_P_RNTI == pdcch.alloc[0].rnti){
                    for(i=0; i<8; i++)
                    {
                        if(rrc_msg.msg[i] != liblte_rrc_test_fill[i])
                        {
                            break;
                        }
                    }
                    if(i == 16)
                    {
                        printf("TEST FILL RECEIVED\n");
                    }else if(LIBLTE_SUCCESS == liblte_rrc_unpack_pcch_msg(&rrc_msg,
                                                                          &pcch_msg)){
                        //print_page(&pcch_msg);
                    }
                }else{
                    printf("MESSAGE RECEIVED FOR RNTI=%04X: ", pdcch.alloc[0].rnti);
                    for(i=0; i<rrc_msg.N_bits; i++)
                    {
                        printf("%u", rrc_msg.msg[i]);
                    }
                    printf("\n");
                }
            }
            // Keep trying to decode PDSCHs
            state            = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC;
            num_samps_needed = phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
            N_sfr++;
            if(N_sfr >= 10)
            {
                N_sfr = 0;
                sfn++;
                samp_buf_r_idx += phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
            }
            break;
        }
    }   

    return (done_flag);
}

void LTE_fdd_dl_fs_samp_buf::freq_shift(uint32 start_idx, uint32 num_samps, float freq_offset)
{
    float  f_samp_re;
    float  f_samp_im;
    float  tmp_i;
    float  tmp_q;
    uint32 i;

    for(i=start_idx; i<(start_idx+num_samps); i++)
    {
        f_samp_re = cosf((i+0)*(freq_offset)*2*M_PI/phy_struct->fs);
        f_samp_im = sinf((i+0)*(freq_offset)*2*M_PI/phy_struct->fs);
        tmp_i     = i_buf[i];
        tmp_q     = q_buf[i];
        i_buf[i]  = tmp_i*f_samp_re + tmp_q*f_samp_im;
        q_buf[i]  = tmp_q*f_samp_re - tmp_i*f_samp_im;
    }
}

void LTE_fdd_dl_fs_samp_buf::freq_shift_Chang(uint32 start_idx, 
                                              uint32 num_samps, 
                                              float  freq_offset)
{
    float  f_samp_re;
    float  f_samp_im;
    float  tmp_i;
    float  tmp_q;
    uint32 i;

    #pragma omp parallel for
    for(i=start_idx; i<(start_idx+num_samps); i++)
    {
        f_samp_re = cosf((i*(freq_offset)*2*M_PI+prev_offset)/phy_struct->fs);
        f_samp_im = sinf((i*(freq_offset)*2*M_PI+prev_offset)/phy_struct->fs);
        tmp_i     = i_buf[i];
        tmp_q     = q_buf[i];
        i_buf[i]  = tmp_i*f_samp_re + tmp_q*f_samp_im;
        q_buf[i]  = tmp_q*f_samp_re - tmp_i*f_samp_im;
    }
}

void LTE_fdd_dl_fs_samp_buf::freq_shift_rcfo(uint32 start_idx, 
                                             uint32  num_samps,
                                             float  *prev_phase,
                                             float   freq_offset)
{
    float  f_samp_re;
    float  f_samp_im;
    float  tmp_i;
    float  tmp_q;
    uint32 i;

    for(i=start_idx; i<(start_idx+num_samps); i++)
    {
        f_samp_re = cosf((i*(freq_offset)*2*M_PI+(*prev_phase))/phy_struct->fs);
        f_samp_im = sinf((i*(freq_offset)*2*M_PI+(*prev_phase))/phy_struct->fs);
        tmp_i     = i_buf[i];
        tmp_q     = q_buf[i];
        i_buf[i]  = tmp_i*f_samp_re + tmp_q*f_samp_im;
        q_buf[i]  = tmp_q*f_samp_re - tmp_i*f_samp_im;
    }
    *prev_phase += freq_offset*num_samps;
}

void LTE_fdd_dl_fs_samp_buf::print_msg(LIBLTE_BIT_MSG_STRUCT *rrc_msg)
{
    printf("DL LTE Shared Channel :\n");
    printf("\tMessage Decoded, %d rv:\n", harq_proc[0].CURRENT_IRV);
    uint8* tmp = rrc_msg->msg;
    printf("\t\t%d%s= ", message_idx,"-th Message");
    for(uint32 i=0; i<rrc_msg->N_bits/8; i++)
    {
        printf("%c",bits_2_value_char(&tmp, 8));
    }
    printf("\n");
}

void LTE_fdd_dl_fs_samp_buf::print_mib(LIBLTE_RRC_MIB_STRUCT *mib)
{
    if(false == mib_printed)
    {
        printf("DL LTE Channel found [%u]:\n", corr_peak_idx);
        printf("\tMIB Decoded:\n");
        printf("\t\t%-40s=%20.2f\n", "Frequency Offset", cfo);
        printf("\t\t%-40s=%20u\n", "System Frame Number", sfn);
        printf("\t\t%-40s=%20u\n", "Physical Cell ID", N_id_cell);
        printf("\t\t%-40s=%20u\n", "Number of TX Antennas", N_ant);
        printf("\t\t%-40s=%17sMHz\n", "Bandwidth", liblte_rrc_dl_bandwidth_text[mib->dl_bw]);
        printf("\t\t%-40s=%20s\n", "PHICH Duration", liblte_rrc_phich_duration_text[mib->phich_config.dur]);
        printf("\t\t%-40s=%20s\n", "PHICH Resource", liblte_rrc_phich_resource_text[mib->phich_config.res]);

        //mib_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib1(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1)
{
    uint32 i;
    uint32 j;
    uint32 si_win_len;
    uint32 si_periodicity_T;
    uint16 mnc;

    if(true              == prev_si_value_tag_valid &&
       prev_si_value_tag != sib1->system_info_value_tag)
    {
        printf("\tSystem Info value tag changed\n");
        sib1_printed = false;
        sib2_printed = false;
        sib3_printed = false;
        sib4_printed = false;
        sib5_printed = false;
        sib6_printed = false;
        sib7_printed = false;
        sib8_printed = false;
    }

    if(false == sib1_printed)
    {
        printf("\tSIB1 Decoded:\n");
        printf("\t\t%-40s\n", "PLMN Identity List:");
        for(i=0; i<sib1->N_plmn_ids; i++)
        {
            printf("\t\t\t%03X-", sib1->plmn_id[i].id.mcc & 0x0FFF);
            if((sib1->plmn_id[i].id.mnc & 0xFF00) == 0xFF00)
            {
                mnc = sib1->plmn_id[i].id.mnc & 0x00FF;
                printf("%02X, ", mnc);
            }else{
                mnc = sib1->plmn_id[i].id.mnc & 0x0FFF;
                printf("%03X, ", mnc);
            }
            for(j=0; j<LIBLTE_MCC_MNC_LIST_N_ITEMS; j++)
            {
                if(liblte_mcc_mnc_list[j].mcc == (sib1->plmn_id[i].id.mcc & 0x0FFF) &&
                   liblte_mcc_mnc_list[j].mnc == mnc)
                {
                    printf("%s, ", liblte_mcc_mnc_list[j].net_name);
                    break;
                }
            }
            if(LIBLTE_RRC_RESV_FOR_OPER == sib1->plmn_id[i].resv_for_oper)
            {
                printf("reserved for operator use\n");
            }else{
                printf("not reserved for operator use\n");
            }
        }
        printf("\t\t%-40s=%20u\n", "Tracking Area Code", sib1->tracking_area_code);
        printf("\t\t%-40s=%20u\n", "Cell Identity", sib1->cell_id);
        switch(sib1->cell_barred)
        {
        case LIBLTE_RRC_CELL_BARRED:
            printf("\t\t%-40s=%20s\n", "Cell Barred", "Barred");
            break;
        case LIBLTE_RRC_CELL_NOT_BARRED:
            printf("\t\t%-40s=%20s\n", "Cell Barred", "Not Barred");
            break;
        }
        switch(sib1->intra_freq_reselection)
        {
        case LIBLTE_RRC_INTRA_FREQ_RESELECTION_ALLOWED:
            printf("\t\t%-40s=%20s\n", "Intra Frequency Reselection", "Allowed");
            break;
        case LIBLTE_RRC_INTRA_FREQ_RESELECTION_NOT_ALLOWED:
            printf("\t\t%-40s=%20s\n", "Intra Frequency Reselection", "Not Allowed");
            break;
        }
        if(true == sib1->csg_indication)
        {
            printf("\t\t%-40s=%20s\n", "CSG Indication", "TRUE");
        }else{
            printf("\t\t%-40s=%20s\n", "CSG Indication", "FALSE");
        }
        if(LIBLTE_RRC_CSG_IDENTITY_NOT_PRESENT != sib1->csg_id)
        {
            printf("\t\t%-40s=%20u\n", "CSG Identity", sib1->csg_id);
        }
        printf("\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib1->q_rx_lev_min);
        printf("\t\t%-40s=%18udB\n", "Q Rx Lev Min Offset", sib1->q_rx_lev_min_offset);
        if(true == sib1->p_max_present)
        {
            printf("\t\t%-40s=%17ddBm\n", "P Max", sib1->p_max);
        }
        printf("\t\t%-40s=%20u\n", "Frequency Band", sib1->freq_band_indicator);
        printf("\t\t%-40s=%18sms\n", "SI Window Length", liblte_rrc_si_window_length_text[sib1->si_window_length]);
        si_win_len = liblte_rrc_si_window_length_num[sib1->si_window_length];
        printf("\t\t%-40s\n", "Scheduling Info List:");
        for(i=0; i<sib1->N_sched_info; i++)
        {
            printf("\t\t\t%s = %s frames\n", "SI Periodicity", liblte_rrc_si_periodicity_text[sib1->sched_info[i].si_periodicity]);
            si_periodicity_T = liblte_rrc_si_periodicity_num[sib1->sched_info[i].si_periodicity];
            printf("\t\t\tSI Window Starts at N_subframe = %u, SFN mod %u = %u\n", (i * si_win_len) % 10, si_periodicity_T, (i * si_win_len)/10);
            if(0 == i)
            {
                printf("\t\t\t\t%s = %s\n", "SIB Type", "2");
            }
            for(j=0; j<sib1->sched_info[i].N_sib_mapping_info; j++)
            {
                printf("\t\t\t\t%s = %u\n", "SIB Type", liblte_rrc_sib_type_num[sib1->sched_info[i].sib_mapping_info[j].sib_type]);
                switch(sib1->sched_info[i].sib_mapping_info[j].sib_type)
                {
                case LIBLTE_RRC_SIB_TYPE_3:
                    sib3_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_4:
                    sib4_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_5:
                    sib5_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_6:
                    sib6_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_7:
                    sib7_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_8:
                    sib8_expected = true;
                    break;
                }
            }
        }
        if(false == sib1->tdd)
        {
            printf("\t\t%-40s=%20s\n", "Duplexing Mode", "FDD");
        }else{
            printf("\t\t%-40s=%20s\n", "Duplexing Mode", "TDD");
            printf("\t\t%-40s=%20s\n", "Subframe Assignment", liblte_rrc_subframe_assignment_text[sib1->sf_assignment]);
            printf("\t\t%-40s=%20s\n", "Special Subframe Patterns", liblte_rrc_special_subframe_patterns_text[sib1->special_sf_patterns]);
        }
        printf("\t\t%-40s=%20u\n", "SI Value Tag", sib1->system_info_value_tag);
        prev_si_value_tag       = sib1->system_info_value_tag;
        prev_si_value_tag_valid = true;

        sib1_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib2(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2)
{
    uint32 coeff = 0;
    uint32 T     = 0;
    uint32 i;

    if(false == sib2_printed)
    {
        printf("\tSIB2 Decoded:\n");
        if(true == sib2->ac_barring_info_present)
        {
            if(true == sib2->ac_barring_for_emergency)
            {
                printf("\t\t%-40s=%20s\n", "AC Barring for Emergency", "Barred");
            }else{
                printf("\t\t%-40s=%20s\n", "AC Barring for Emergency", "Not Barred");
            }
            if(true == sib2->ac_barring_for_mo_signalling.enabled)
            {
                printf("\t\t%-40s=%20s\n", "AC Barring for MO Signalling", "Barred");
                printf("\t\t\t%-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_signalling.factor]);
                printf("\t\t\t%-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_signalling.time]);
                printf("\t\t\t%-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_signalling.for_special_ac);
            }else{
                printf("\t\t%-40s=%20s\n", "AC Barring for MO Signalling", "Not Barred");
            }
            if(true == sib2->ac_barring_for_mo_data.enabled)
            {
                printf("\t\t%-40s=%20s\n", "AC Barring for MO Data", "Barred");
                printf("\t\t\t%-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_data.factor]);
                printf("\t\t\t%-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_data.time]);
                printf("\t\t\t%-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_data.for_special_ac);
            }else{
                printf("\t\t%-40s=%20s\n", "AC Barring for MO Data", "Not Barred");
            }
        }
        printf("\t\t%-40s=%20s\n", "Number of RACH Preambles", liblte_rrc_number_of_ra_preambles_text[sib2->rr_config_common_sib.rach_cnfg.num_ra_preambles]);
        if(true == sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present)
        {
            printf("\t\t%-40s=%20s\n", "Size of RACH Preambles Group A", liblte_rrc_size_of_ra_preambles_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.size_of_ra]);
            printf("\t\t%-40s=%15s bits\n", "Message Size Group A", liblte_rrc_message_size_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_size]);
            printf("\t\t%-40s=%18sdB\n", "Message Power Offset Group B", liblte_rrc_message_power_offset_group_b_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_pwr_offset_group_b]);
        }
        printf("\t\t%-40s=%18sdB\n", "Power Ramping Step", liblte_rrc_power_ramping_step_text[sib2->rr_config_common_sib.rach_cnfg.pwr_ramping_step]);
        printf("\t\t%-40s=%17sdBm\n", "Preamble init target RX power", liblte_rrc_preamble_initial_received_target_power_text[sib2->rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr]);
        printf("\t\t%-40s=%20s\n", "Preamble TX Max", liblte_rrc_preamble_trans_max_text[sib2->rr_config_common_sib.rach_cnfg.preamble_trans_max]);
        printf("\t\t%-40s=%10s Subframes\n", "RA Response Window Size", liblte_rrc_ra_response_window_size_text[sib2->rr_config_common_sib.rach_cnfg.ra_resp_win_size]);
        printf("\t\t%-40s=%10s Subframes\n", "MAC Contention Resolution Timer", liblte_rrc_mac_contention_resolution_timer_text[sib2->rr_config_common_sib.rach_cnfg.mac_con_res_timer]);
        printf("\t\t%-40s=%20u\n", "Max num HARQ TX for Message 3", sib2->rr_config_common_sib.rach_cnfg.max_harq_msg3_tx);
        printf("\t\t%-40s=%20s\n", "Modification Period Coeff", liblte_rrc_modification_period_coeff_text[sib2->rr_config_common_sib.bcch_cnfg.modification_period_coeff]);
        coeff = liblte_rrc_modification_period_coeff_num[sib2->rr_config_common_sib.bcch_cnfg.modification_period_coeff];
        printf("\t\t%-40s=%13s Frames\n", "Default Paging Cycle", liblte_rrc_default_paging_cycle_text[sib2->rr_config_common_sib.pcch_cnfg.default_paging_cycle]);
        T = liblte_rrc_default_paging_cycle_num[sib2->rr_config_common_sib.pcch_cnfg.default_paging_cycle];
        printf("\t\t%-40s=%13u Frames\n", "Modification Period", coeff * T);
        printf("\t\t%-40s=%13u Frames\n", "nB", (uint32)(T * liblte_rrc_nb_num[sib2->rr_config_common_sib.pcch_cnfg.nB]));
        printf("\t\t%-40s=%20u\n", "Root Sequence Index", sib2->rr_config_common_sib.prach_cnfg.root_sequence_index);
        printf("\t\t%-40s=%20u\n", "PRACH Config Index", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index);
        switch(sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index)
        {
        case 0:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 1:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 2:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 3:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 4:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 5:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 6:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 7:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 8:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 9:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 10:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 11:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 12:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 13:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 14:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Any, RACH Subframe Number = 0,1,2,3,4,5,6,7,8,9\n");
            break;
        case 15:
            printf("\t\t\tPreamble Format = 0, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 16:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 17:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 18:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 19:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 20:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 21:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 22:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 23:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 24:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 25:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 26:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 27:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 28:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 29:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 30:
            printf("\t\t\tPreamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 31:
            printf("\t\t\tPreamble Format = 1, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 32:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 33:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 34:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 35:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 36:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 37:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 38:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 39:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 40:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 41:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 42:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 43:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 44:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 45:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 46:
            printf("\t\t\tPreamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 47:
            printf("\t\t\tPreamble Format = 2, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 48:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 49:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 50:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 51:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 52:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 53:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 54:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 55:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 56:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 57:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 58:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 59:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 60:
            printf("\t\t\tPreamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 61:
            printf("\t\t\tPreamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 62:
            printf("\t\t\tPreamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 63:
            printf("\t\t\tPreamble Format = 3, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        }
        if(true == sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag)
        {
            printf("\t\t%-40s=%20s\n", "High Speed Flag", "Restricted Set");
        }else{
            printf("\t\t%-40s=%20s\n", "High Speed Flag", "Unrestricted Set");
        }
        printf("\t\t%-40s=%20u\n", "Ncs Configuration", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config);
        printf("\t\t%-40s=%20u\n", "PRACH Freq Offset", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset);
        printf("\t\t%-40s=%17ddBm\n", "Reference Signal Power", sib2->rr_config_common_sib.pdsch_cnfg.rs_power);
        printf("\t\t%-40s=%20u\n", "Pb", sib2->rr_config_common_sib.pdsch_cnfg.p_b);
        printf("\t\t%-40s=%20u\n", "Nsb", sib2->rr_config_common_sib.pusch_cnfg.n_sb);
        switch(sib2->rr_config_common_sib.pusch_cnfg.hopping_mode)
        {
        case LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME:
            printf("\t\t%-40s=%20s\n", "Hopping Mode", "Inter Subframe");
            break;
        case LIBLTE_RRC_HOPPING_MODE_INTRA_AND_INTER_SUBFRAME:
            printf("\t\t%-40s= %s\n", "Hopping Mode", "Intra and Inter Subframe");
            break;
        }
        printf("\t\t%-40s=%20u\n", "PUSCH Nrb Hopping Offset", sib2->rr_config_common_sib.pusch_cnfg.pusch_hopping_offset);
        if(true == sib2->rr_config_common_sib.pusch_cnfg.enable_64_qam)
        {
            printf("\t\t%-40s=%20s\n", "64QAM", "Allowed");
        }else{
            printf("\t\t%-40s=%20s\n", "64QAM", "Not Allowed");
        }
        if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled)
        {
            printf("\t\t%-40s=%20s\n", "Group Hopping", "Enabled");
        }else{
            printf("\t\t%-40s=%20s\n", "Group Hopping", "Disabled");
        }
        printf("\t\t%-40s=%20u\n", "Group Assignment PUSCH", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch);
        if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled)
        {
            printf("\t\t%-40s=%20s\n", "Sequence Hopping", "Enabled");
        }else{
            printf("\t\t%-40s=%20s\n", "Sequence Hopping", "Disabled");
        }
        printf("\t\t%-40s=%20u\n", "Cyclic Shift", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift);
        printf("\t\t%-40s=%20s\n", "Delta PUCCH Shift", liblte_rrc_delta_pucch_shift_text[sib2->rr_config_common_sib.pucch_cnfg.delta_pucch_shift]);
        printf("\t\t%-40s=%20u\n", "N_rb_cqi", sib2->rr_config_common_sib.pucch_cnfg.n_rb_cqi);
        printf("\t\t%-40s=%20u\n", "N_cs_an", sib2->rr_config_common_sib.pucch_cnfg.n_cs_an);
        printf("\t\t%-40s=%20u\n", "N1 PUCCH AN", sib2->rr_config_common_sib.pucch_cnfg.n1_pucch_an);
        if(true == sib2->rr_config_common_sib.srs_ul_cnfg.present)
        {
            printf("\t\t%-40s=%20s\n", "SRS Bandwidth Config", liblte_rrc_srs_bw_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.bw_cnfg]);
            printf("\t\t%-40s=%20s\n", "SRS Subframe Config", liblte_rrc_srs_subfr_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.subfr_cnfg]);
            if(true == sib2->rr_config_common_sib.srs_ul_cnfg.ack_nack_simul_tx)
            {
                printf("\t\t%-40s=%20s\n", "Simultaneous AN and SRS", "True");
            }else{
                printf("\t\t%-40s=%20s\n", "Simultaneous AN and SRS", "False");
            }
            if(true == sib2->rr_config_common_sib.srs_ul_cnfg.max_up_pts_present)
            {
                printf("\t\t%-40s=%20s\n", "SRS Max Up PTS", "True");
            }else{
                printf("\t\t%-40s=%20s\n", "SRS Max Up PTS", "False");
            }
        }
        printf("\t\t%-40s=%17ddBm\n", "P0 Nominal PUSCH", sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch);
        printf("\t\t%-40s=%20s\n", "Alpha", liblte_rrc_ul_power_control_alpha_text[sib2->rr_config_common_sib.ul_pwr_ctrl.alpha]);
        printf("\t\t%-40s=%17ddBm\n", "P0 Nominal PUCCH", sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch);
        printf("\t\t%-40s=%18sdB\n", "Delta F PUCCH Format 1", liblte_rrc_delta_f_pucch_format_1_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1]);
        printf("\t\t%-40s=%18sdB\n", "Delta F PUCCH Format 1B", liblte_rrc_delta_f_pucch_format_1b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b]);
        printf("\t\t%-40s=%18sdB\n", "Delta F PUCCH Format 2", liblte_rrc_delta_f_pucch_format_2_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2]);
        printf("\t\t%-40s=%18sdB\n", "Delta F PUCCH Format 2A", liblte_rrc_delta_f_pucch_format_2a_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a]);
        printf("\t\t%-40s=%18sdB\n", "Delta F PUCCH Format 2B", liblte_rrc_delta_f_pucch_format_2b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b]);
        printf("\t\t%-40s=%18ddB\n", "Delta Preamble Message 3", sib2->rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3);
        switch(sib2->rr_config_common_sib.ul_cp_length)
        {
        case LIBLTE_RRC_UL_CP_LENGTH_1:
            printf("\t\t%-40s=%20s\n", "UL CP Length", "Normal");
            break;
        case LIBLTE_RRC_UL_CP_LENGTH_2:
            printf("\t\t%-40s=%20s\n", "UL CP Length", "Extended");
            break;
        }
        printf("\t\t%-40s=%18sms\n", "T300", liblte_rrc_t300_text[sib2->ue_timers_and_constants.t300]);
        printf("\t\t%-40s=%18sms\n", "T301", liblte_rrc_t301_text[sib2->ue_timers_and_constants.t301]);
        printf("\t\t%-40s=%18sms\n", "T310", liblte_rrc_t310_text[sib2->ue_timers_and_constants.t310]);
        printf("\t\t%-40s=%20s\n", "N310", liblte_rrc_n310_text[sib2->ue_timers_and_constants.n310]);
        printf("\t\t%-40s=%18sms\n", "T311", liblte_rrc_t311_text[sib2->ue_timers_and_constants.t311]);
        printf("\t\t%-40s=%20s\n", "N311", liblte_rrc_n311_text[sib2->ue_timers_and_constants.n311]);
        if(true == sib2->arfcn_value_eutra.present)
        {
            printf("\t\t%-40s=%20u\n", "UL ARFCN", sib2->arfcn_value_eutra.value);
        }
        if(true == sib2->ul_bw.present)
        {
            printf("\t\t%-40s=%17sMHz\n", "UL Bandwidth", liblte_rrc_ul_bw_text[sib2->ul_bw.bw]);
        }
        printf("\t\t%-40s=%20u\n", "Additional Spectrum Emission", sib2->additional_spectrum_emission);
        if(0 != sib2->mbsfn_subfr_cnfg_list_size)
        {
            printf("\t\t%s:\n", "MBSFN Subframe Config List");
        }
        for(i=0; i<sib2->mbsfn_subfr_cnfg_list_size; i++)
        {
            printf("\t\t\t%-40s=%20s\n", "Radio Frame Alloc Period", liblte_rrc_radio_frame_allocation_period_text[sib2->mbsfn_subfr_cnfg[i].radio_fr_alloc_period]);
            printf("\t\t\t%-40s=%20u\n", "Radio Frame Alloc Offset", sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
            printf("\t\t\tSubframe Alloc%-26s=%20u\n", liblte_rrc_subframe_allocation_num_frames_text[sib2->mbsfn_subfr_cnfg[i].subfr_alloc_num_frames], sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
        }
        printf("\t\t%-40s=%10s Subframes\n", "Time Alignment Timer", liblte_rrc_time_alignment_timer_text[sib2->time_alignment_timer]);

        sib2_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib3(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3)
{
    if(false == sib3_printed)
    {
        printf("\tSIB3 Decoded:\n");
        printf("\t\t%-40s=%18sdB\n", "Q-Hyst", liblte_rrc_q_hyst_text[sib3->q_hyst]);
        if(true == sib3->speed_state_resel_params.present)
        {
            printf("\t\t%-40s=%19ss\n", "T-Evaluation", liblte_rrc_t_evaluation_text[sib3->speed_state_resel_params.mobility_state_params.t_eval]);
            printf("\t\t%-40s=%19ss\n", "T-Hyst Normal", liblte_rrc_t_hyst_normal_text[sib3->speed_state_resel_params.mobility_state_params.t_hyst_normal]);
            printf("\t\t%-40s=%20u\n", "N-Cell Change Medium", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_medium);
            printf("\t\t%-40s=%20u\n", "N-Cell Change High", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_high);
            printf("\t\t%-40s=%18sdB\n", "Q-Hyst SF Medium", liblte_rrc_sf_medium_text[sib3->speed_state_resel_params.q_hyst_sf.medium]);
            printf("\t\t%-40s=%18sdB\n", "Q-Hyst SF High", liblte_rrc_sf_high_text[sib3->speed_state_resel_params.q_hyst_sf.high]);
        }
        if(true == sib3->s_non_intra_search_present)
        {
            printf("\t\t%-40s=%18udB\n", "S-Non Intra Search", sib3->s_non_intra_search);
        }
        printf("\t\t%-40s=%18udB\n", "Threshold Serving Low", sib3->thresh_serving_low);
        printf("\t\t%-40s=%20u\n", "Cell Reselection Priority", sib3->cell_resel_prio);
        printf("\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib3->q_rx_lev_min);
        if(true == sib3->p_max_present)
        {
            printf("\t\t%-40s=%17ddBm\n", "P Max", sib3->p_max);
        }
        if(true == sib3->s_intra_search_present)
        {
            printf("\t\t%-40s=%18udB\n", "S-Intra Search", sib3->s_intra_search);
        }
        if(true == sib3->allowed_meas_bw_present)
        {
            printf("\t\t%-40s=%17sMHz\n", "Allowed Meas Bandwidth", liblte_rrc_allowed_meas_bandwidth_text[sib3->allowed_meas_bw]);
        }
        if(true == sib3->presence_ant_port_1)
        {
            printf("\t\t%-40s=%20s\n", "Presence Antenna Port 1", "True");
        }else{
            printf("\t\t%-40s=%20s\n", "Presence Antenna Port 1", "False");
        }
        switch(sib3->neigh_cell_cnfg)
        {
        case 0:
            printf("\t\t%-40s= %s\n", "Neighbor Cell Config", "Not all neighbor cells have the same MBSFN alloc");
            break;
        case 1:
            printf("\t\t%-40s= %s\n", "Neighbor Cell Config", "MBSFN allocs are identical for all neighbor cells");
            break;
        case 2:
            printf("\t\t%-40s= %s\n", "Neighbor Cell Config", "No MBSFN allocs are present in neighbor cells");
            break;
        case 3:
            printf("\t\t%-40s= %s\n", "Neighbor Cell Config", "Different UL/DL allocs in neighbor cells for TDD");
            break;
        }
        printf("\t\t%-40s=%19us\n", "T-Reselection EUTRA", sib3->t_resel_eutra);
        if(true == sib3->t_resel_eutra_sf_present)
        {
            printf("\t\t%-40s=%20s\n", "T-Reselection EUTRA SF Medium", liblte_rrc_sssf_medium_text[sib3->t_resel_eutra_sf.sf_medium]);
            printf("\t\t%-40s=%20s\n", "T-Reselection EUTRA SF High", liblte_rrc_sssf_high_text[sib3->t_resel_eutra_sf.sf_high]);
        }

        sib3_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib4(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4)
{
    uint32 i;
    uint32 stop;

    if(false == sib4_printed)
    {
        printf("\tSIB4 Decoded:\n");
        if(0 != sib4->intra_freq_neigh_cell_list_size)
        {
            printf("\t\tList of intra-frequency neighboring cells:\n");
        }
        for(i=0; i<sib4->intra_freq_neigh_cell_list_size; i++)
        {
            printf("\t\t\t%s = %u\n", "Physical Cell ID", sib4->intra_freq_neigh_cell_list[i].phys_cell_id);
            printf("\t\t\t\t%s = %sdB\n", "Q Offset Range", liblte_rrc_q_offset_range_text[sib4->intra_freq_neigh_cell_list[i].q_offset_range]);
        }
        if(0 != sib4->intra_freq_black_cell_list_size)
        {
            printf("\t\tList of blacklisted intra-frequency neighboring cells:\n");
        }
        for(i=0; i<sib4->intra_freq_black_cell_list_size; i++)
        {
            printf("\t\t\t%u - %u\n", sib4->intra_freq_black_cell_list[i].start, sib4->intra_freq_black_cell_list[i].start + liblte_rrc_phys_cell_id_range_num[sib4->intra_freq_black_cell_list[i].range]);
        }
        if(true == sib4->csg_phys_cell_id_range_present)
        {
            printf("\t\t%-40s= %u - %u\n", "CSG Phys Cell ID Range", sib4->csg_phys_cell_id_range.start, sib4->csg_phys_cell_id_range.start + liblte_rrc_phys_cell_id_range_num[sib4->csg_phys_cell_id_range.range]);
        }

        sib4_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib5(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5)
{
    uint32 i;
    uint32 j;
    uint16 stop;

    if(false == sib5_printed)
    {
        printf("\tSIB5 Decoded:\n");
        printf("\t\tList of inter-frequency neighboring cells:\n");
        for(i=0; i<sib5->inter_freq_carrier_freq_list_size; i++)
        {
            printf("\t\t\t%-40s=%20u\n", "ARFCN", sib5->inter_freq_carrier_freq_list[i].dl_carrier_freq);
            printf("\t\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib5->inter_freq_carrier_freq_list[i].q_rx_lev_min);
            if(true == sib5->inter_freq_carrier_freq_list[i].p_max_present)
            {
                printf("\t\t\t%-40s=%17ddBm\n", "P Max", sib5->inter_freq_carrier_freq_list[i].p_max);
            }
            printf("\t\t\t%-40s=%19us\n", "T-Reselection EUTRA", sib5->inter_freq_carrier_freq_list[i].t_resel_eutra);
            if(true == sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf_present)
            {
                printf("\t\t\t%-40s=%20s\n", "T-Reselection EUTRA SF Medium", liblte_rrc_sssf_medium_text[sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf.sf_medium]);
                printf("\t\t\t%-40s=%20s\n", "T-Reselection EUTRA SF High", liblte_rrc_sssf_high_text[sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf.sf_high]);
            }
            printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib5->inter_freq_carrier_freq_list[i].threshx_high);
            printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib5->inter_freq_carrier_freq_list[i].threshx_low);
            printf("\t\t\t%-40s=%17sMHz\n", "Allowed Meas Bandwidth", liblte_rrc_allowed_meas_bandwidth_text[sib5->inter_freq_carrier_freq_list[i].allowed_meas_bw]);
            if(true == sib5->inter_freq_carrier_freq_list[i].presence_ant_port_1)
            {
                printf("\t\t\t%-40s=%20s\n", "Presence Antenna Port 1", "True");
            }else{
                printf("\t\t\t%-40s=%20s\n", "Presence Antenna Port 1", "False");
            }
            if(true == sib5->inter_freq_carrier_freq_list[i].cell_resel_prio_present)
            {
                printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib5->inter_freq_carrier_freq_list[i].cell_resel_prio);
            }
            switch(sib5->inter_freq_carrier_freq_list[i].neigh_cell_cnfg)
            {
            case 0:
                printf("\t\t\t%-40s= %s\n", "Neighbor Cell Config", "Not all neighbor cells have the same MBSFN alloc");
                break;
            case 1:
                printf("\t\t\t%-40s= %s\n", "Neighbor Cell Config", "MBSFN allocs are identical for all neighbor cells");
                break;
            case 2:
                printf("\t\t\t%-40s= %s\n", "Neighbor Cell Config", "No MBSFN allocs are present in neighbor cells");
                break;
            case 3:
                printf("\t\t\t%-40s= %s\n", "Neighbor Cell Config", "Different UL/DL allocs in neighbor cells for TDD");
                break;
            }
            printf("\t\t\t%-40s=%18sdB\n", "Q Offset Freq", liblte_rrc_q_offset_range_text[sib5->inter_freq_carrier_freq_list[i].q_offset_freq]);
            if(0 != sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list_size)
            {
                printf("\t\t\tList of inter-frequency neighboring cells with specific cell reselection parameters:\n");
                for(j=0; j<sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list_size; j++)
                {
                    printf("\t\t\t\t%-40s=%20u\n", "Physical Cell ID", sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].phys_cell_id);
                    printf("\t\t\t\t%-40s=%18sdB\n", "Q Offset Cell", liblte_rrc_q_offset_range_text[sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].q_offset_cell]);
                }
            }
            if(0 != sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size)
            {
                printf("\t\t\tList of blacklisted inter-frequency neighboring cells\n");
                for(j=0; j<sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size; j++)
                {
                    printf("\t\t\t\t%u - %u\n", sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start, sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start + liblte_rrc_phys_cell_id_range_num[sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].range]);
                }
            }
        }

        sib5_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib6(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6)
{
    uint32 i;

    if(false == sib6_printed)
    {
        printf("\tSIB6 Decoded:\n");
        if(0 != sib6->carrier_freq_list_utra_fdd_size)
        {
            printf("\t\t%s:\n", "Carrier Freq List UTRA FDD");
        }
        for(i=0; i<sib6->carrier_freq_list_utra_fdd_size; i++)
        {
            printf("\t\t\t%-40s=%20u\n", "ARFCN", sib6->carrier_freq_list_utra_fdd[i].carrier_freq);
            if(true == sib6->carrier_freq_list_utra_fdd[i].cell_resel_prio_present)
            {
                printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib6->carrier_freq_list_utra_fdd[i].cell_resel_prio);
            }
            printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib6->carrier_freq_list_utra_fdd[i].threshx_high);
            printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib6->carrier_freq_list_utra_fdd[i].threshx_low);
            printf("\t\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib6->carrier_freq_list_utra_fdd[i].q_rx_lev_min);
            printf("\t\t\t%-40s=%17ddBm\n", "P Max UTRA", sib6->carrier_freq_list_utra_fdd[i].p_max_utra);
            printf("\t\t\t%-40s=%18dB\n", "Q Qual Min", sib6->carrier_freq_list_utra_fdd[i].q_qual_min);
        }
        if(0 != sib6->carrier_freq_list_utra_tdd_size)
        {
            printf("\t\t%s:\n", "Carrier Freq List UTRA TDD");
        }
        for(i=0; i<sib6->carrier_freq_list_utra_tdd_size; i++)
        {
            printf("\t\t\t%-40s=%20u\n", "ARFCN", sib6->carrier_freq_list_utra_tdd[i].carrier_freq);
            if(true == sib6->carrier_freq_list_utra_tdd[i].cell_resel_prio_present)
            {
                printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib6->carrier_freq_list_utra_tdd[i].cell_resel_prio);
            }
            printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib6->carrier_freq_list_utra_tdd[i].threshx_high);
            printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib6->carrier_freq_list_utra_tdd[i].threshx_low);
            printf("\t\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib6->carrier_freq_list_utra_tdd[i].q_rx_lev_min);
            printf("\t\t\t%-40s=%17ddBm\n", "P Max UTRA", sib6->carrier_freq_list_utra_tdd[i].p_max_utra);
        }
        printf("\t\t%-40s=%19us\n", "T-Reselection UTRA", sib6->t_resel_utra);
        if(true == sib6->t_resel_utra_sf_present)
        {
            printf("\t\t%-40s=%20s\n", "T-Reselection UTRA SF Medium", liblte_rrc_sssf_medium_text[sib6->t_resel_utra_sf.sf_medium]);
            printf("\t\t%-40s=%20s\n", "T-Reselection UTRA SF High", liblte_rrc_sssf_high_text[sib6->t_resel_utra_sf.sf_high]);
        }

        sib6_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib7(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7)
{
    uint32 i;
    uint32 j;

    if(false == sib7_printed)
    {
        printf("\tSIB7 Decoded:\n");
        printf("\t\t%-40s=%19us\n", "T-Reselection GERAN", sib7->t_resel_geran);
        if(true == sib7->t_resel_geran_sf_present)
        {
            printf("\t\t%-40s=%20s\n", "T-Reselection GERAN SF Medium", liblte_rrc_sssf_medium_text[sib7->t_resel_geran_sf.sf_medium]);
            printf("\t\t%-40s=%20s\n", "T-Reselection GERAN SF High", liblte_rrc_sssf_high_text[sib7->t_resel_geran_sf.sf_high]);
        }
        if(0 != sib7->carrier_freqs_info_list_size)
        {
            printf("\t\tList of neighboring GERAN carrier frequencies\n");
        }
        for(i=0; i<sib7->carrier_freqs_info_list_size; i++)
        {
            printf("\t\t\t%-40s=%20u\n", "Starting ARFCN", sib7->carrier_freqs_info_list[i].carrier_freqs.starting_arfcn);
            printf("\t\t\t%-40s=%20s\n", "Band Indicator", liblte_rrc_band_indicator_geran_text[sib7->carrier_freqs_info_list[i].carrier_freqs.band_indicator]);
            if(LIBLTE_RRC_FOLLOWING_ARFCNS_EXPLICIT_LIST == sib7->carrier_freqs_info_list[i].carrier_freqs.following_arfcns)
            {
                printf("\t\t\tFollowing ARFCNs Explicit List\n");
                for(j=0; j<sib7->carrier_freqs_info_list[i].carrier_freqs.explicit_list_of_arfcns_size; j++)
                {
                    printf("\t\t\t\t%u\n", sib7->carrier_freqs_info_list[i].carrier_freqs.explicit_list_of_arfcns[j]);
                }
            }else if(LIBLTE_RRC_FOLLOWING_ARFCNS_EQUALLY_SPACED == sib7->carrier_freqs_info_list[i].carrier_freqs.following_arfcns){
                printf("\t\t\tFollowing ARFCNs Equally Spaced\n");
                printf("\t\t\t\t%u, %u\n", sib7->carrier_freqs_info_list[i].carrier_freqs.equally_spaced_arfcns.arfcn_spacing, sib7->carrier_freqs_info_list[i].carrier_freqs.equally_spaced_arfcns.number_of_arfcns);
            }else{
                printf("\t\t\tFollowing ARFCNs Variable Bit Map\n");
                printf("\t\t\t\t%02X\n", sib7->carrier_freqs_info_list[i].carrier_freqs.variable_bit_map_of_arfcns);
            }
            if(true == sib7->carrier_freqs_info_list[i].cell_resel_prio_present)
            {
                printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib7->carrier_freqs_info_list[i].cell_resel_prio);
            }
            printf("\t\t\t%-40s=%20u\n", "NCC Permitted", sib7->carrier_freqs_info_list[i].ncc_permitted);
            printf("\t\t\t%-40s=%17ddBm\n", "Q Rx Lev Min", sib7->carrier_freqs_info_list[i].q_rx_lev_min);
            if(true == sib7->carrier_freqs_info_list[i].p_max_geran_present)
            {
                printf("\t\t\t%-40s=%17udBm\n", "P Max GERAN", sib7->carrier_freqs_info_list[i].p_max_geran);
            }
            printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib7->carrier_freqs_info_list[i].threshx_high);
            printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib7->carrier_freqs_info_list[i].threshx_low);
        }

        sib7_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_sib8(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8)
{
    uint32 i;
    uint32 j;
    uint32 k;

    if(false == sib8_printed)
    {
        printf("\tSIB8 Decoded:\n");
        if(true == sib8->sys_time_info_present)
        {
            if(true == sib8->sys_time_info_cdma2000.cdma_eutra_sync)
            {
                printf("\t\t%-40s=%20s\n", "CDMA EUTRA sync", "True");
            }else{
                printf("\t\t%-40s=%20s\n", "CDMA EUTRA sync", "False");
            }
            if(true == sib8->sys_time_info_cdma2000.system_time_async)
            {
                printf("\t\t%-40s=%14llu chips\n", "System Time", sib8->sys_time_info_cdma2000.system_time * 8);
            }else{
                printf("\t\t%-40s=%17llu ms\n", "System Time", sib8->sys_time_info_cdma2000.system_time * 10);
            }
        }
        if(true == sib8->search_win_size_present)
        {
            printf("\t\t%-40s=%20u\n", "Search Window Size", sib8->search_win_size);
        }
        if(true == sib8->params_hrpd_present)
        {
            if(true == sib8->pre_reg_info_hrpd.pre_reg_allowed)
            {
                printf("\t\t%-40s=%20s\n", "Pre Registration", "Allowed");
            }else{
                printf("\t\t%-40s=%20s\n", "Pre Registration", "Not Allowed");
            }
            if(true == sib8->pre_reg_info_hrpd.pre_reg_zone_id_present)
            {
                printf("\t\t%-40s=%20u\n", "Pre Registration Zone ID", sib8->pre_reg_info_hrpd.pre_reg_zone_id);
            }
            if(0 != sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size)
            {
                printf("\t\tSecondary Pre Registration Zone IDs:\n");
            }
            for(i=0; i<sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size; i++)
            {
                printf("\t\t\t%u\n", sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list[i]);
            }
            if(true == sib8->cell_resel_params_hrpd_present)
            {
                printf("\t\tBand Class List:\n");
                for(i=0; i<sib8->cell_resel_params_hrpd.band_class_list_size; i++)
                {
                    printf("\t\t\t%-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.band_class_list[i].band_class]);
                    if(true == sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio_present)
                    {
                        printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio);
                    }
                    printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_high);
                    printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_low);
                }
                printf("\t\tNeighbor Cell List:\n");
                for(i=0; i<sib8->cell_resel_params_hrpd.neigh_cell_list_size; i++)
                {
                    printf("\t\t\t%-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.neigh_cell_list[i].band_class]);
                    printf("\t\t\tNeighbor Cells Per Frequency List\n");
                    for(j=0; j<sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++)
                    {
                        printf("\t\t\t\t%-40s=%20u\n", "ARFCN", sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                        printf("\t\t\t\tPhys Cell ID List\n");
                        for(k=0; k<sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++)
                        {
                            printf("\t\t\t\t\t%u\n", sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list[k]);
                        }
                    }
                }
                printf("\t\t%-40s=%19us\n", "T Reselection", sib8->cell_resel_params_hrpd.t_resel_cdma2000);
                if(true == sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf_present)
                {
                    printf("\t\t%-40s=%20s\n", "T-Reselection Scale Factor Medium", liblte_rrc_sssf_medium_text[sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf.sf_medium]);
                    printf("\t\t%-40s=%20s\n", "T-Reselection Scale Factor High", liblte_rrc_sssf_high_text[sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf.sf_high]);
                }
            }
        }
        if(true == sib8->params_1xrtt_present)
        {
            printf("\t\tCSFB Registration Parameters\n");
            if(true == sib8->csfb_reg_param_1xrtt_present)
            {
                printf("\t\t\t%-40s=%20u\n", "SID", sib8->csfb_reg_param_1xrtt.sid);
                printf("\t\t\t%-40s=%20u\n", "NID", sib8->csfb_reg_param_1xrtt.nid);
                if(true == sib8->csfb_reg_param_1xrtt.multiple_sid)
                {
                    printf("\t\t\t%-40s=%20s\n", "Multiple SIDs", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Multiple SIDs", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.multiple_nid)
                {
                    printf("\t\t\t%-40s=%20s\n", "Multiple NIDs", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Multiple NIDs", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.home_reg)
                {
                    printf("\t\t\t%-40s=%20s\n", "Home Reg", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Home Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.foreign_sid_reg)
                {
                    printf("\t\t\t%-40s=%20s\n", "Foreign SID Reg", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Foreign SID Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.foreign_nid_reg)
                {
                    printf("\t\t\t%-40s=%20s\n", "Foreign NID Reg", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Foreign NID Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.param_reg)
                {
                    printf("\t\t\t%-40s=%20s\n", "Parameter Reg", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Parameter Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.power_up_reg)
                {
                    printf("\t\t\t%-40s=%20s\n", "Power Up Reg", "True");
                }else{
                    printf("\t\t\t%-40s=%20s\n", "Power Up Reg", "False");
                }
                printf("\t\t\t%-40s=%20u\n", "Registration Period", sib8->csfb_reg_param_1xrtt.reg_period);
                printf("\t\t\t%-40s=%20u\n", "Registration Zone", sib8->csfb_reg_param_1xrtt.reg_zone);
                printf("\t\t\t%-40s=%20u\n", "Total Zones", sib8->csfb_reg_param_1xrtt.total_zone);
                printf("\t\t\t%-40s=%20u\n", "Zone Timer", sib8->csfb_reg_param_1xrtt.zone_timer);
            }
            if(true == sib8->long_code_state_1xrtt_present)
            {
                printf("\t\t%-40s=%20llu\n", "Long Code State", sib8->long_code_state_1xrtt);
            }
            if(true == sib8->cell_resel_params_1xrtt_present)
            {
                printf("\t\tBand Class List:\n");
                for(i=0; i<sib8->cell_resel_params_1xrtt.band_class_list_size; i++)
                {
                    printf("\t\t\t%-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.band_class_list[i].band_class]);
                    if(true == sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio_present)
                    {
                        printf("\t\t\t%-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio);
                    }
                    printf("\t\t\t%-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_high);
                    printf("\t\t\t%-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_low);
                }
                printf("\t\tNeighbor Cell List:\n");
                for(i=0; i<sib8->cell_resel_params_1xrtt.neigh_cell_list_size; i++)
                {
                    printf("\t\t\t%-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.neigh_cell_list[i].band_class]);
                    printf("\t\t\tNeighbor Cells Per Frequency List\n");
                    for(j=0; j<sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++)
                    {
                        printf("\t\t\t\t%-40s=%20u\n", "ARFCN", sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                        printf("\t\t\t\tPhys Cell ID List\n");
                        for(k=0; k<sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++)
                        {
                            printf("\t\t\t\t\t%u\n", sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list[k]);
                        }
                    }
                }
                printf("\t\t%-40s=%19us\n", "T Reselection", sib8->cell_resel_params_1xrtt.t_resel_cdma2000);
                if(true == sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf_present)
                {
                    printf("\t\t%-40s=%20s\n", "T-Reselection Scale Factor Medium", liblte_rrc_sssf_medium_text[sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf.sf_medium]);
                    printf("\t\t%-40s=%20s\n", "T-Reselection Scale Factor High", liblte_rrc_sssf_high_text[sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf.sf_high]);
                }
            }
        }

        sib8_printed = true;
    }
}

void LTE_fdd_dl_fs_samp_buf::print_config(void)
{
    uint32 i;

    printf("***System Configuration Parameters***\n");
    printf("\tType 'help' to reprint this menu\n");
    printf("\tHit enter to finish config and scan file\n");
    printf("\tSet parameters using <param>=<value> format\n");

    // FS
    printf("\t%-30s = %10s, values = [",
           FS_PARAM,
           liblte_phy_fs_text[fs]);
    for(i=0; i<LIBLTE_PHY_FS_N_ITEMS; i++)
    {
        if(0 != i)
        {
            printf(", ");
        }
        printf("%s", liblte_phy_fs_text[i]);
    }
    printf("]\n");
}

void LTE_fdd_dl_fs_samp_buf::wrap_phase(float *phase_1, float  phase_2)
{
    while((*phase_1 - phase_2) > M_PI)
    {
        *phase_1 = *phase_1 - 2*M_PI;
    }
    while((*phase_1 - phase_2) < -M_PI)
    {
        *phase_1 = *phase_1 + 2*M_PI;
    }
}

void LTE_fdd_dl_fs_samp_buf::wrap_offset(float *offset_1, float  offset_2)
{
    while((*offset_1) >= offset_2)
    {
        *offset_1 = *offset_1 - offset_2;
    }
    while((*offset_1) <= -offset_2)
    {
        *offset_1 = *offset_1 + offset_2;
    }
}

void LTE_fdd_dl_fs_samp_buf::change_config(char *line)
{
    char *param;
    char *value;
    bool  err = false;

    param = strtok(line, "=");
    value = strtok(NULL, "=");

    if(param == NULL)
    {
        need_config = false;
    }else{
        if(!strcasecmp(param, "help"))
        {
            print_config();
        }else if(value != NULL){
            if(!strcasecmp(param, FS_PARAM))
            {
                err = set_fs(value);
            }else{
                printf("Invalid parameter (%s)\n", param);
            }

            if(err)
            {
                printf("Invalid value\n");
            }
        }else{
            printf("Invalid value\n");
        }
    }
}

bool LTE_fdd_dl_fs_samp_buf::set_fs(char *char_value)
{
    bool err = false;

    if(!strcasecmp(char_value, "30.72"))
    {
        fs = LIBLTE_PHY_FS_30_72MHZ;
    }else if(!strcasecmp(char_value, "15.36")){
        fs = LIBLTE_PHY_FS_15_36MHZ;
    }else if(!strcasecmp(char_value, "7.68")){
        fs = LIBLTE_PHY_FS_7_68MHZ;
    }else if(!strcasecmp(char_value, "3.84")){
        fs = LIBLTE_PHY_FS_3_84MHZ;
    }else if(!strcasecmp(char_value, "1.92")){
        fs = LIBLTE_PHY_FS_1_92MHZ;
    }else{
        err = true;
    }

    return(err);
}

void ul_offset_calculation(LIBLTE_PHY_STRUCT               *phy_struct,
                           LIBLTE_PHY_SUBFRAME_STRUCT      *subframe,
                           LIBLTE_PHY_ALLOCATION_STRUCT    **alloc, 
                           uint32                           num_users,
                           float                           *freq_offset_u
                          )
{
    float*  dmrs_0_re;
    float*  dmrs_0_im;
    float*  dmrs_1_re;
    float*  dmrs_1_im;
    float  tmp_re_0;
    float  tmp_im_0;
    float  tmp_re_1;
    float  tmp_im_1;
    float  tmp_phase;
    float  tmp_phase_re;
    float  tmp_phase_im;
    uint32  i;
    uint32  L;
    uint32  user;
    uint32  M_pusch_sc;
    uint32  N_samps_per_OFDM_sym = phy_struct->N_samps_per_symb + phy_struct->N_samps_cp_l_else;

    for(user=0; user<num_users; user++)
    {
        M_pusch_sc = alloc[user]->N_prb * phy_struct->N_sc_rb_ul;

        dmrs_0_re = (phy_struct->dmrs_0_re[subframe->num][alloc[user]->N_prb]);
        dmrs_0_im = (phy_struct->dmrs_0_im[subframe->num][alloc[user]->N_prb]);
        dmrs_1_re = (phy_struct->dmrs_1_re[subframe->num][alloc[user]->N_prb]);
        dmrs_1_im = (phy_struct->dmrs_1_im[subframe->num][alloc[user]->N_prb]);

        tmp_phase = 0;
        for(i=0; i<M_pusch_sc; i++)
        {
            /// Channel gain of DMRS 0
            tmp_re_0 = subframe->rx_symb_re[3][alloc[user]->prb[0][0]*12+i]*dmrs_0_re[i]
                      +subframe->rx_symb_im[3][alloc[user]->prb[0][0]*12+i]*dmrs_0_im[i];
            tmp_im_0 = subframe->rx_symb_im[3][alloc[user]->prb[0][0]*12+i]*dmrs_0_re[i]
                      -subframe->rx_symb_re[3][alloc[user]->prb[0][0]*12+i]*dmrs_0_im[i];

            /// Channel gain of DMRS 1          
            tmp_re_1 = subframe->rx_symb_re[10][alloc[user]->prb[0][0]*12+i]*dmrs_1_re[i]
                      +subframe->rx_symb_im[10][alloc[user]->prb[0][0]*12+i]*dmrs_1_im[i];
            tmp_im_1 = subframe->rx_symb_im[10][alloc[user]->prb[0][0]*12+i]*dmrs_1_re[i]
                      -subframe->rx_symb_re[10][alloc[user]->prb[0][0]*12+i]*dmrs_1_im[i];

            tmp_phase_re = tmp_re_1*tmp_re_0 + tmp_im_1*tmp_im_0;
            tmp_phase_im = tmp_im_1*tmp_re_0 - tmp_re_1*tmp_im_0;
            tmp_phase += atan2f(tmp_phase_im, tmp_phase_re);
        }
        freq_offset_u[user] = (tmp_phase*15000)/(2*M_PI*(7*N_samps_per_OFDM_sym)*M_pusch_sc/(phy_struct->N_samps_per_symb));
        fprintf(stderr, "user %d's freq offset is %f...\n",user, freq_offset_u[user]);
    }
}

void ul_cfo_sco_compensation(LIBLTE_PHY_STRUCT               *phy_struct,
                             float                            NCO_phase,
                             float                            freq_offset_u,
                             float                           *i_buf,
                             float                           *q_buf,
                             float                           *compensate_i_buf,
                             float                           *compensate_q_buf
                            )
{
    uint32  i;
    float   phase;
    float   f_samp_re;
    float   f_samp_im;
    float   tmp_i;
    float   tmp_q;

    for(i=0; i<phy_struct->N_samps_per_subfr; i++)
    {
        f_samp_re = cosf(NCO_phase+(i+0)*(freq_offset_u)*2*M_PI/phy_struct->fs);
        f_samp_im = sinf(NCO_phase+(i+0)*(freq_offset_u)*2*M_PI/phy_struct->fs);
        tmp_i     = i_buf[i];
        tmp_q     = q_buf[i];
        compensate_i_buf[i]  = tmp_i*f_samp_re + tmp_q*f_samp_im;
        compensate_q_buf[i]  = tmp_q*f_samp_re - tmp_i*f_samp_im;
    }
}

extern void generate_prs_c(uint32  c_init,
                           uint32  len,
                           uint32 *c);

void LTE_fdd_dl_fs_samp_buf::phich_channel_decode(LIBLTE_PHY_STRUCT*                phy_struct,
                                                  LIBLTE_PHY_ALLOCATION_STRUCT*     alloc,
                                                  LIBLTE_PHY_SUBFRAME_STRUCT        *subframe,
                                                  uint32                            N_id_cell,
                                                  float                             phich_res,
                                                  LIBLTE_RRC_PHICH_DURATION_ENUM    phich_dur,
                                                  uint8                             *ACK_or_NACK                
                                                 )
{
    uint32 M_layer_symb;
    uint32 M_symb;
    uint32 m_prime;
    uint32 l_prime;
    uint32 n_l_prime;
    uint32 c_init;
    uint32 n_hat[3];
    uint32 i;
    uint32 j;
    uint32 p;
    uint32 idx;
    uint32 re_idx;
    uint32 I_prb_ra;
    uint32 n_group_phich;
    uint32 n_seq_phich;
    uint32 w_idx;
    uint32 y_idx;
    uint32 z_idx;

    float  tmp_ack_re[12];
    float  tmp_ack_im[12];
    float  tmp_nack_re[12];
    float  tmp_nack_im[12];
    float  tmp_re;
    float  tmp_im;
    /// record Euclidean distance
    float  dist_to_ack  = 0;
    float  dist_to_nack = 0;

    float PHICH_w_re_normal_cp_6_9_1_2[8][4] = {{ 1, 1, 1, 1},
                                                { 1,-1, 1,-1},
                                                { 1, 1,-1,-1},
                                                { 1,-1,-1, 1},
                                                { 0, 0, 0, 0},
                                                { 0, 0, 0, 0},
                                                { 0, 0, 0, 0},
                                                { 0, 0, 0, 0}};
    float PHICH_w_im_normal_cp_6_9_1_2[8][4] = {{ 0, 0, 0, 0},
                                                { 0, 0, 0, 0},
                                                { 0, 0, 0, 0},
                                                { 0, 0, 0, 0},
                                                { 1, 1, 1, 1},
                                                { 1,-1, 1,-1},
                                                { 1, 1,-1,-1},
                                                { 1,-1,-1, 1}};


    c_init = (((subframe->num + 1)*(2*N_id_cell + 1)) << 9) + N_id_cell;
    generate_prs_c(c_init, 12, phy_struct->pdcch_c);

    I_prb_ra      = alloc->prb[0][0];
    n_group_phich = I_prb_ra % phy_struct->N_group_phich;
    n_seq_phich   = (I_prb_ra/phy_struct->N_group_phich) % (2*phy_struct->N_sf_phich);

    for(i=0; i<(3 * phy_struct->N_sf_phich); i++)
    {
        // De-scrambling
        if(phy_struct->pdcch_c[i] == 1)
        {
            phy_struct->phich_d_re[i] = phy_struct->phich_d_re[i]*(-1);
            phy_struct->phich_d_im[i] = phy_struct->phich_d_im[i]*(-1);
        }

        // Mutiply Orthogonal Code
        w_idx = i % phy_struct->N_sf_phich;
        z_idx = i / phy_struct->N_sf_phich;

        tmp_re = phy_struct->phich_d_re[i];
        tmp_im = phy_struct->phich_d_im[i];
        phy_struct->phich_d_re[i] = PHICH_w_re_normal_cp_6_9_1_2[n_seq_phich][w_idx]*tmp_re 
                                  - PHICH_w_im_normal_cp_6_9_1_2[n_seq_phich][w_idx]*tmp_im;
        phy_struct->phich_d_im[i] = PHICH_w_re_normal_cp_6_9_1_2[n_seq_phich][w_idx]*tmp_im 
                                  + PHICH_w_im_normal_cp_6_9_1_2[n_seq_phich][w_idx]*tmp_re;
                
        tmp_ack_re[i] = (n_seq_phich < 4) ? phy_struct->bpsk_ack_re[z_idx] : -1*phy_struct->bpsk_ack_re[z_idx];
        tmp_ack_im[i] = (n_seq_phich < 4) ? phy_struct->bpsk_ack_re[z_idx] : -1*phy_struct->bpsk_ack_re[z_idx]; 
                      
        tmp_nack_re[i] = (n_seq_phich < 4) ? phy_struct->bpsk_nack_re[z_idx] : -1*phy_struct->bpsk_nack_re[z_idx];
        tmp_nack_im[i] = (n_seq_phich < 4) ? phy_struct->bpsk_nack_re[z_idx] : -1*phy_struct->bpsk_nack_re[z_idx]; 

        
        dist_to_ack += (phy_struct->phich_d_re[i]-tmp_ack_re[i])*(phy_struct->phich_d_re[i]-tmp_ack_re[i])              
                     + (phy_struct->phich_d_im[i]-tmp_ack_im[i])*(phy_struct->phich_d_im[i]-tmp_ack_im[i]);
        dist_to_nack += (phy_struct->phich_d_re[i]-tmp_nack_re[i])*(phy_struct->phich_d_re[i]-tmp_nack_re[i])              
                      + (phy_struct->phich_d_im[i]-tmp_nack_im[i])*(phy_struct->phich_d_im[i]-tmp_nack_im[i]); 
    }  

    *ACK_or_NACK = (dist_to_ack < dist_to_nack) ? 1 : 0;
}

/// Radio class, created by Chia-Hao Chang

Radio* Radio::get_instance(void)
{
    if(instance == NULL)
    {
        instance = new Radio();
    }
    return instance; 
}

Radio::Radio()
{
    init();
    recv_buff   = std::vector<std::complex<float> >(19200, std::complex<float>(0.0, 0.0));
    send_buff   = std::vector<std::complex<float> >(19200, std::complex<float>(0.0, 0.0));
    started     = true;
}

void* Radio::recv_to_buffer(void* inputs)
{
    LTE_fdd_dl_fs_samp_buf* UE       = LTE_fdd_dl_fs_samp_buf::get_instance();
    LTE_FDD_ENB_RADIO_TX_BUF_STRUCT  tx_radio_buf[2];
    LTE_FDD_ENB_RADIO_RX_BUF_STRUCT  rx_radio_buf[2];
    struct timespec                  sleep_time;
    struct timespec                  time_rem;
    struct sched_param               priority;
    uhd::rx_metadata_t               metadata;
    uhd::time_spec_t                 next_rx_ts;
    uhd::time_spec_t                 next_rx_subfr_ts;
    uhd::time_spec_t                 check_ts;
    uhd::stream_cmd_t                cmd = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    int64                            next_rx_ts_ticks;
    int64                            metadata_ts_ticks;
    uint32                           i;
    uint32                           N_subfrs_dropped;
    uint32                           error          = 0;
    uint32                           recv_size      = 1920*5;
    uint32                           samp_rate      = 1.92*1e6;
    uint32                           buf_idx        = 0;
    uint32                           recv_idx       = 0;
    uint32                           samp_idx       = 0;
    uint32                           num_samps      = 0;
    uint32                           radio_idx      = 0;
    uint16                           rx_current_tti = (LTE_FDD_ENB_CURRENT_TTI_MAX + 1) - 2;
    bool                             init_needed    = true;
    bool                             rx_synced      = false;

    // Assign UE C_RNTI
    UE->C_RNTI = this->external_c_rnti;

    while(started)
    {
        if(init_needed)
        {
            // Setup time specs
            //next_tx_ts       = uhd::time_spec_t::from_ticks(samp_rate, samp_rate);
            //next_rx_ts       = next_tx_ts;
            //next_rx_ts      -= uhd::time_spec_t::from_ticks(1920*2, samp_rate);
            //next_rx_subfr_ts = next_rx_ts;

            // Reset USRP time
            usrp->set_time_now(uhd::time_spec_t::from_ticks(0, samp_rate));

            // Start streaming 
            cmd.stream_now   = true;
            usrp->issue_stream_cmd(cmd);

            init_needed      = false;
        }
        if(!rx_synced)
        {
            // recv half frame to synchronize the timing
            // num_samps = rx_stream->recv(recv_buff, recv_size, metadata);
            num_samps = usrp->get_device()->recv(&recv_buff.front(), recv_size, rx_md, 
                                                 uhd::io_type_t::COMPLEX_FLOAT32,
                                                 uhd::device::RECV_MODE_FULL_BUFF);
            tx_md.time_spec  = rx_md.time_spec+uhd::time_spec_t::from_ticks(num_samps, samp_rate);
            tx_md.time_spec += uhd::time_spec_t::from_ticks(1920, samp_rate);;
            if(0 != num_samps || (recv_size==0&&0==num_samps))
            {
                if(num_samps < recv_size)
                {
                    recv_size = recv_size-num_samps;
                    for(i=0; i<num_samps; i++)
                    {
                        rx_radio_buf[buf_idx].i_buf[samp_idx+i] = recv_buff[recv_idx+i].real();
                        rx_radio_buf[buf_idx].q_buf[samp_idx+i] = recv_buff[recv_idx+i].imag();
                    }
                    samp_idx += num_samps;
                }else{
                    if(UE->recv_state == PBCH_DECODE_STATE)
                    {
                        recv_size   = 1920;
                        rx_synced   = true;

                        // Start PBCH decoding and synchronization is done.
                        send_from_buffer(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
                        num_samps = usrp->get_device()->recv(&recv_buff.front(), 1920, rx_md, 
                                                             uhd::io_type_t::COMPLEX_FLOAT32,
                                                             uhd::device::RECV_MODE_FULL_BUFF);
                    }else{
                        #pragma omp parallel for
                        for(i=0; i<num_samps; i++)
                        {
                            rx_radio_buf[buf_idx].i_buf[samp_idx+i] = recv_buff[recv_idx+i].real();
                            rx_radio_buf[buf_idx].q_buf[samp_idx+i] = recv_buff[recv_idx+i].imag();
                        }
                        samp_idx    = 0;
                        recv_size   = UE->cell_search(rx_radio_buf[buf_idx].i_buf, rx_radio_buf[buf_idx].q_buf, 9600);
                    }
                }
            }
        }else{
            if(0 != num_samps)
            {
                #pragma omp parallel for
                for(i=0; i<num_samps; i++)
                {
                    rx_radio_buf[buf_idx].i_buf[samp_idx+i] = recv_buff[recv_idx+i].real();
                    rx_radio_buf[buf_idx].q_buf[samp_idx+i] = recv_buff[recv_idx+i].imag();
                }
                
                error = UE->execute(rx_radio_buf[buf_idx].i_buf, rx_radio_buf[buf_idx].q_buf, 1920);
                
                // PBCH decode failed... need to back to cell search state.
                if(error==-1 && UE->recv_state==CELL_SEARCH_STATE) 
                {
                    rx_synced = false;
                    recv_size = 1920*5;
                    continue;
                }
            }    
            // num_samps = rx_stream->recv(recv_buff, recv_size, metadata);
            // already sync, tx should start transmitting.
            num_samps = usrp->get_device()->recv(&recv_buff.front(), 1920, rx_md, 
                                                 uhd::io_type_t::COMPLEX_FLOAT32,
                                                 uhd::device::RECV_MODE_FULL_BUFF);
            //tx_md.time_spec  = rx_md.time_spec;
            //tx_md.time_spec += uhd::time_spec_t::from_ticks(1920, samp_rate);
        }
    }
}

void* Radio::send_from_buffer(float* i_data, float* q_data, uint32 len)
{
    tx_md.has_time_spec     = true;
    tx_md.start_of_burst    = false;
    tx_md.end_of_burst      = false;

    #pragma omp parallel for
    for(uint32 i=0; i<len; i++)
    {
        send_buff[i] = std::complex<float>(i_data[i]/50.0, q_data[i]/50.0);
    }
    // before sending tx_md should set its time_spec_t
    tx_stream->send(&send_buff.front(), 1920, tx_md, 0.001);
    tx_md.time_spec += uhd::time_spec_t::from_ticks(1920, 1.92*1e6);
}

void Radio::init()
{   
    // Initial system 
    uhd::set_thread_priority_safe();

    devs                = uhd::device::find(hint);
    usrp                = uhd::usrp::multi_usrp::make(devs[0]); // device_addrs_t

    init_usrp(1.92*1e6, 0.4*1e9, 40, false);
    uhd::stream_args_t    stream_args("fc32");

    tx_stream  = usrp->get_tx_stream(stream_args);
    rx_stream  = usrp->get_rx_stream(stream_args);
}

void Radio::init_usrp(double rate, double freq, double gain, bool clock) 
{
    
    usrp->set_clock_source("internal"); // or mimo
    //usrp->set_time_source("internal");  // or mimo
    usrp->set_rx_rate(rate);
    usrp->set_tx_rate(rate);
    usrp->set_rx_freq(freq);
    usrp->set_tx_freq(freq);
    usrp->set_rx_gain(gain);
    // sync to external clock
    if(clock) {
        usrp->set_clock_config(uhd::clock_config_t::external());
        usrp->set_time_next_pps(uhd::time_spec_t(0.0));
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); //allow for some setup time
}

void* Radio::recv_from_mq(void* inputs)
{
    LTE_fdd_dl_fs_samp_buf* UE       = LTE_fdd_dl_fs_samp_buf::get_instance();
    MessageQueue*           mq       = MessageQueue::get_instance();

    LTE_FDD_ENB_RADIO_TX_BUF_STRUCT  tx_radio_buf[2];
    LTE_FDD_ENB_RADIO_RX_BUF_STRUCT  rx_radio_buf[2];
    struct timespec                  sleep_time;
    struct timespec                  time_rem;
    struct sched_param               priority;
    uhd::rx_metadata_t               metadata;
    uhd::time_spec_t                 next_rx_ts;
    uhd::time_spec_t                 next_rx_subfr_ts;
    uhd::time_spec_t                 check_ts;
    uhd::stream_cmd_t                cmd = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
    int64                            next_rx_ts_ticks;
    int64                            metadata_ts_ticks;
    uint32                           i;
    uint32                           N_subfrs_dropped;
    uint32                           error          = 0;
    uint32                           recv_size      = 1920*5;
    uint32                           samp_rate      = 1.92*1e6;
    uint32                           buf_idx        = 0;
    uint32                           recv_idx       = 0;
    uint32                           samp_idx       = 0;
    uint32                           num_samps      = 0;
    uint32                           radio_idx      = 0;
    uint16                           rx_current_tti = (LTE_FDD_ENB_CURRENT_TTI_MAX + 1) - 2;
    float                            I_Buf[19200]   ;
    float                            Q_Buf[19200]   ;
    bool                             init_needed    = true;
    bool                             rx_synced      = false;
    
    // Assign UE C_RNTI
    UE->C_RNTI          = this->external_c_rnti;
    UE->pdcch.c_rnti    = UE->C_RNTI;

    while(started)
    {
        if(!rx_synced)
        {
            num_samps = mq->dl_recv_from_mq(I_Buf, Q_Buf, recv_size);

            if(0!=num_samps || (recv_size==0&&0==num_samps))
            {
                if(num_samps < recv_size)
                {
                    recv_size = recv_size-num_samps;
                    samp_idx += num_samps;
                }else{
                    if(UE->recv_state == PBCH_DECODE_STATE)
                    {
                        recv_size   = 1920;
                        rx_synced   = true;

                        // Start PBCH decoding and synchronization is done.
                        mq->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
                        mq->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
                        num_samps = mq->dl_recv_from_mq(I_Buf, Q_Buf, 1920);

                    }else{
                        samp_idx    = 0;
                        recv_size   = UE->cell_search(I_Buf, Q_Buf, 9600);
                    }
                }
            }
        }else{
            if(0 != num_samps)
            {
                cerr<<"-----------*************-----------"<<endl;
                cerr<<"-----------Execute Stage-----------"<<endl;
                cerr<<"-----------*************-----------"<<endl;
                //getchar();
                error = UE->mq_execute(I_Buf, Q_Buf, 1920);
                
                // PBCH decode failed... need to back to cell search state.
                if(error==-1 && UE->recv_state==CELL_SEARCH_STATE) 
                {
                    rx_synced = false;
                    recv_size = 1920*5;
                    continue;
                }
            }    
            num_samps = mq->dl_recv_from_mq(I_Buf, Q_Buf, 1920);
        }
    }
}
   

uint32 LTE_fdd_dl_fs_samp_buf::mq_execute(float* i_data, float* q_data, uint32 len)
{
    MessageQueue* ul_radio = MessageQueue::get_instance();
    switch(recv_state)
    {
        case CELL_SEARCH_STATE:
            cell_search(i_data, q_data, len);
            break;
        case PBCH_DECODE_STATE: 
            ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            pbch_decoding(i_data, q_data, len);    
            if(get_ul_init() == false)
            {
                ul_process();
            }
            break;
        case PDCCH_DECODE_STATE:
            cerr << "Now is PDCCH Decode State, subframe num:"<< current_tti%10 << endl;
            //getchar();
            ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            if(current_tti%10 == 5)
            {
                sib1_decoding(i_data, q_data, len);
            }else if(current_tti%10 != 0){
                dlsch_decoding(i_data, q_data, len, current_tti%10);
            }else{
                tracking(i_data, q_data, len);
            }
            break;
        case PUCCH_REPORT_STATE:
                if(time_to_op-2 == current_tti)
                {
                    float ul_i_buf[1920];
                    float ul_q_buf[1920];

                    // Uplink Control Channel
                    pucch_encoding(ul_i_buf, ul_q_buf, current_tti%10);

                    // Send to BS
                    ul_radio->ul_send_to_mq(ul_i_buf, ul_q_buf, 1920);
                }else{
                    ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
                }
                tracking(i_data, q_data, len);
            break;
        case PUSCH_ENCODE_STATE:
            if(time_to_op-2 == current_tti)
            {
                float ul_i_buf[1920];
                float ul_q_buf[1920];

                // Uplink Shared Channel 
                ulsch_encoding(ul_i_buf, ul_q_buf, len);

                ul_radio->ul_send_to_mq(ul_i_buf, ul_q_buf, 1920);
            }else{
                ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            }    
            tracking(i_data, q_data, len);
            
            break;
        case PHICH_DECODE_STATE:
            ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
            if(time_to_op == current_tti)
            {
                phich_decoding(i_data, q_data, len, current_tti%10);
            }else{
                tracking(i_data, q_data, len);
            }
            break;

        case WATING_STATE:
            ul_radio->ul_send_to_mq(ZERO_SIGNAL, ZERO_SIGNAL, 1920);
  
            if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_DLSCH)
            {
                if(time_to_op-1 == current_tti)
                    recv_state              = PDCCH_DECODE_STATE;
            }else if(Alloc_Info.chan_type == LIBLTE_PHY_CHAN_TYPE_ULSCH){
                if(time_to_op-3 == current_tti && !ack_or_nack)
                {
                    recv_state              = PUSCH_ENCODE_STATE;
                }else if(ack_or_nack){
                    recv_state              = PDCCH_DECODE_STATE;
                    Alloc_Info.chan_type    = UNDEFINED_CHAN_TYPE;
                }
            }
            
            tracking(i_data, q_data, len);
            break;
        default:
            cerr << "Undefined State..." << endl;
            break;    
    }
}

void LTE_fdd_dl_fs_samp_buf::set_ul_msg(void)
{
    // Get the information bits

    uint8* tmp = Alloc_Info.msg.msg;
    value_2_bits(tx_information, &tmp, 8);
    value_2_bits(tx_information, &tmp, 8);
    value_2_bits(tx_information, &tmp, 8);
    value_2_bits(tx_information, &tmp, 8);

    if(tx_information>='A' || tx_information<='Z')
    {
        tx_information -= 1;
    }else{
        tx_information = 'Z';
    }
    Alloc_Info.msg.N_bits   = 32;
    Alloc_Info.tbs          = 32;
}
