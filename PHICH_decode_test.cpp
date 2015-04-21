
	LIBLTE_PHY_STRUCT          	   *phy_struct = new LIBLTE_PHY_STRUCT;
	LIBLTE_PHY_SUBFRAME_STRUCT*       subframe = new LIBLTE_PHY_SUBFRAME_STRUCT;
	LIBLTE_PHY_PCFICH_STRUCT                 pcfich;
    LIBLTE_PHY_PHICH_STRUCT                  phich;
    uint32            N_bits;

	liblte_phy_init(&phy_struct,
                    LIBLTE_PHY_FS_1_92MHZ,
                    299,
                    1,
                    6,
                    12,
                    1);
	float i_buf[19200];
	float q_buf[19200];
	LTE_File Read_USRP_RX("OFDM_symbol.dat", READ);
	Read_USRP_RX.LTE_File_Read(i_buf, q_buf, 19200);
	liblte_phy_get_dl_subframe_and_ce(phy_struct,
                                      i_buf,
                                      q_buf,
                                      0,
                                      0,
                                      299,
                                      4,
                                      subframe);
	subframe->num=0;
	pcfich_channel_demap(phy_struct, subframe, 299, 1, &pcfich, &N_bits);
	phich_channel_demap(phy_struct, 
						&pcfich, 
						subframe, 
						299, 
						1, 
						1, 
						LIBLTE_RRC_PHICH_DURATION_NORMAL, 
						&phich);
	LIBLTE_PHY_ALLOCATION_STRUCT*     alloc = new LIBLTE_PHY_ALLOCATION_STRUCT;
	uint8                             ACK_or_NACK ;
	alloc->prb[0][0] = 0;
	phich_channel_decode(phy_struct,
                         alloc,
                         subframe,
                         299,
                         1,
                         LIBLTE_RRC_PHICH_DURATION_NORMAL,
                         &ACK_or_NACK); 
	cout << ACK_or_NACK << endl;
	}