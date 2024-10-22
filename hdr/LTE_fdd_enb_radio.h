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

    File: LTE_fdd_enb_radio.h

    Description: Contains all the definitions for the LTE FDD eNodeB
                 radio device control.

    Revision History
    ----------    -------------    --------------------------------------------
    11/09/2013    Ben Wojtowicz    Created file
    01/18/2014    Ben Wojtowicz    Handling EARFCN updates and multiple
                                   antennas.
    06/15/2014    Ben Wojtowicz    Changed fn_combo to current_tti.
    07/22/2014    Ben Wojtowicz    Added clock source as a configurable
                                   parameter.

*******************************************************************************/

#ifndef __LTE_FDD_ENB_RADIO_H__
#define __LTE_FDD_ENB_RADIO_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "LTE_fdd_enb_interface.h"
#include "LTE_fdd_enb_cnfg_db.h"                              
#include "liblte_phy.h"
#include <gnuradio/gr_complex.h>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <boost/thread/mutex.hpp>
#include <stdlib.h>
/*******************************************************************************
                              DEFINES
*******************************************************************************/
#define LTE_FDD_ENB_CURRENT_TTI_MAX (LIBLTE_PHY_SFN_MAX*10 + 9)

/*******************************************************************************
                              FORWARD DECLARATIONS
*******************************************************************************/
class LTE_fdd_enb_phy; // from LTE_fdd_main.h

/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/

typedef struct{
    std::string name;
}LTE_FDD_ENB_RADIO_STRUCT;

typedef struct{
    LTE_FDD_ENB_RADIO_STRUCT radio[100];
    uint32                   num_radios;
}LTE_FDD_ENB_AVAILABLE_RADIOS_STRUCT;

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

/*******************************************************************************
                              CLASS DECLARATIONS
*******************************************************************************/

class LTE_fdd_enb_radio
{
public:
    // Singleton
    static LTE_fdd_enb_radio* get_instance(void);
    static void cleanup(void);

    // Start/Stop
    bool is_started(void);
    LTE_FDD_ENB_ERROR_ENUM start(void);
    LTE_FDD_ENB_ERROR_ENUM start_for_1_92_MHz(void);
    LTE_FDD_ENB_ERROR_ENUM stop(void);

    // External interface
    LTE_FDD_ENB_AVAILABLE_RADIOS_STRUCT get_available_radios(void);
    LTE_FDD_ENB_RADIO_STRUCT get_selected_radio(void);
    uint32 get_selected_radio_idx(void);
    LTE_FDD_ENB_ERROR_ENUM set_selected_radio_idx(uint32 idx);
    uint32 get_tx_gain(void);
    LTE_FDD_ENB_ERROR_ENUM set_tx_gain(uint32 gain);
    uint32 get_rx_gain(void);
    LTE_FDD_ENB_ERROR_ENUM set_rx_gain(uint32 gain);
    std::string get_clock_source(void);
    LTE_FDD_ENB_ERROR_ENUM set_clock_source(std::string source);
    uint32 get_sample_rate(void);
    void set_sample_rate(uint32 _fs);
    void set_earfcns(int64 dl_earfcn, int64 ul_earfcn);
    void send(LTE_FDD_ENB_RADIO_TX_BUF_STRUCT *buf);

    void wait_radio_thread(void);

    uhd::tx_streamer::sptr              tx_stream;
    uhd::rx_streamer::sptr              rx_stream;

    // Buffer 
    gr_complex       tx_buf[LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    gr_complex       rx_buf[LIBLTE_PHY_N_SAMPS_PER_SUBFR_30_72MHZ];
    uhd::time_spec_t next_tx_ts;
    uint32           fs;

    // Radios
    uhd::usrp::multi_usrp::sptr         usrp_0; // RX
    uhd::usrp::multi_usrp::sptr         usrp_1; // TX

private:
    // Singleton
    static LTE_fdd_enb_radio *instance;
    LTE_fdd_enb_radio();
    ~LTE_fdd_enb_radio();

    // Start/Stop
    boost::mutex start_mutex;
    bool         started;

    
    
    LTE_FDD_ENB_AVAILABLE_RADIOS_STRUCT available_radios;
    uint32                              selected_radio_idx;

    // Radio thread
    static void*     radio_thread_func(void *inputs);
    pthread_t        radio_thread;
    
    
    std::string      clock_source;
    int64            N_ant;
    uint32           N_tx_samps;
    uint32           N_rx_samps;
    uint32           N_samps_per_subfr;
    
    uint32           tx_gain;
    uint32           rx_gain;
    uint16           next_tx_current_tti;
};

#endif /* __LTE_FDD_ENB_RADIO_H__ */
