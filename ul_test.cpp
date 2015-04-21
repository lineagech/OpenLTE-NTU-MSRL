/*************  Test **********************/
    //LIBLTE_PHY_STRUCT            *phy_struct = new LIBLTE_PHY_STRUCT;
    //LIBLTE_PHY_SUBFRAME_STRUCT   *subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
    //subframe->num                  = 0;
    //uint8                        b = 1;
    //uint8                        M_bit = 1;
    //uint8                        N_ant = 1;
    //uint32                       delta_PUCCH_shift = 1;
    //uint32                       N_id_cell = 299;
    //uint32                       N_slot = 0;

    //liblte_phy_init(&phy_struct,
    //              LIBLTE_PHY_FS_1_92MHZ,
    //              299,
    //              1,
    //              6,
    //              12,
    //              0);//phich_res)

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

    ul_subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;

    liblte_phy_pucch_formats_1_1a_1b_encode(phy_struct,
                                          &b, 
                                          M_bit,
                                          N_ant,
                                          delta_PUCCH_shift, 
                                          N_id_cell,
                                          N_slot,  
                                          subframe);
    liblte_phy_create_ul_subframe(phy_struct,
                                  subframe,
                                  0,
                                  Tmp_TX_User1.i_buf[0],
                                  Tmp_TX_User1.q_buf[0]);
    LTE_File UL_TX("ul_user1.dat", WRITE);
    UL_TX.record_OFDM(Tmp_TX_User1.i_buf[0], Tmp_TX_User1.q_buf[0], phy_struct->N_samps_per_subfr);
    UL_TX.close_file();
    UL_TX.~LTE_File();
    
    liblte_phy_get_ul_subframe(phy_struct,
                             Tmp_TX_User1.i_buf[0],
                             Tmp_TX_User1.q_buf[0],
                             subframe);
    liblte_phy_pucch_formats_1_1a_1b_decode(phy_struct,
                                          M_bit,  
                                          N_ant,
                                          delta_PUCCH_shift, 
                                          N_id_cell,
                                          N_slot,  
                                          subframe);
    
    LIBLTE_PHY_SUBFRAME_STRUCT   *subframe2 = new LIBLTE_PHY_SUBFRAME_STRUCT;
    subframe2->num                  = 0;
    LIBLTE_PHY_ALLOCATION_STRUCT *alloc = new LIBLTE_PHY_ALLOCATION_STRUCT;

    alloc->N_prb                   = 2;
    alloc->N_layers                 = 1;
    memset(alloc->msg.msg, 1, 32);
    alloc->msg.N_bits               = 32;
    alloc->tbs                   = 32;
    alloc->mcs              = 0;
    alloc->tx_mode              = 1;
    alloc->rv_idx           = 0;
    alloc->rnti             = LIBLTE_MAC_C_RNTI_START;
    alloc->mod_type         = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    alloc->N_codewords      = 1;
    alloc->prb[0][0]        = 3; alloc->prb[0][1]        = 4;
    alloc->prb[1][0]        = 3; alloc->prb[1][1]        = 4;
    liblte_phy_pusch_channel_encode(phy_struct,
                                    alloc,
                                    N_id_cell,
                                    N_ant,
                                    subframe2);
    
    liblte_phy_create_ul_subframe(phy_struct,
                                  subframe2,
                                  0, /// ant
                                  Tmp_TX_User2.i_buf[0],
                                  Tmp_TX_User2.q_buf[0]);
    LTE_File UL_TX2("ul_user2.dat", WRITE);
    UL_TX2.record_OFDM(Tmp_TX_User2.i_buf[0], Tmp_TX_User2.q_buf[0], phy_struct->N_samps_per_subfr);
    UL_TX2.close_file();
    UL_TX2.~LTE_File();


    for(i=0; i<phy_struct->N_samps_per_subfr; i++)
    {
        Tmp_RX_BS.i_buf[0][i]=Tmp_TX_User1.i_buf[0][i]+Tmp_TX_User2.i_buf[0][i];
        Tmp_RX_BS.q_buf[0][i]=Tmp_TX_User1.q_buf[0][i]+Tmp_TX_User2.q_buf[0][i];
    }
  
    LTE_File Read_data("ul_channel_out.dat", READ);
    Read_data.LTE_File_Read(Tmp_RX_BS.i_buf[0], Tmp_RX_BS.q_buf[0], 1920);




    LIBLTE_PHY_COARSE_TIMING_STRUCT         timing_struct;
    liblte_phy_dl_find_coarse_timing_and_freq_offset_Chang(phy_struct,
                                                           Tmp_RX_BS.i_buf[0],
                                                           Tmp_RX_BS.q_buf[0],
                                                           20, /// 20 symbols weighted, fixed by Chia-Hao Chang
                                                           &timing_struct);
    liblte_phy_get_ul_subframe(phy_struct,
                               &Tmp_RX_BS.i_buf[0][timing_struct.symb_starts[0][0]],
                               &Tmp_RX_BS.q_buf[0][timing_struct.symb_starts[0][0]],
                               subframe);
    
    uint8  out_bits[1024];
    uint32 N_out_bits;
    float  freq_offset_u[1024];
    

    ul_offset_calculation(phy_struct,
                          subframe,
                          &alloc, 
                          1,
                          freq_offset_u);
    ul_cfo_sco_compensation(phy_struct,
                            0,// NCO_phase
                            freq_offset_u[0],
                            Tmp_RX_BS.i_buf[0],
                            Tmp_RX_BS.q_buf[0],
                            Tmp_RX_U1.i_buf[0],
                            Tmp_RX_U1.q_buf[0]
                            );
    liblte_phy_get_ul_subframe(phy_struct,
                               &Tmp_RX_U1.i_buf[0][timing_struct.symb_starts[0][0]],
                               &Tmp_RX_U1.q_buf[0][timing_struct.symb_starts[0][0]],
                               subframe);

    liblte_phy_pucch_formats_1_1a_1b_decode(phy_struct,
                                            M_bit,  
                                            N_ant,
                                            delta_PUCCH_shift, 
                                            N_id_cell,
                                            N_slot,  
                                            subframe);

    liblte_phy_pusch_channel_decode(phy_struct,
                                    subframe,
                                    alloc,
                                    N_id_cell,
                                    N_ant,
                                    out_bits,
                                    &N_out_bits);
  
    fprintf(stderr, "%s\n", out_bits);
    cout<<"N_bits: "<<N_out_bits<<endl;
  
  

    //LTE_File UL_TX("SC_FDMA_symbol.dat", WRITE);
    //UL_TX.record_OFDM(Tmp_TX.i_buf[0], Tmp_TX.q_buf[0], phy_struct->N_samps_per_subfr);
    //UL_TX.close_file();
    /*****/