/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 * Copyright 2016 IMDEA Networks Institute
 *
 * \section LICENSE
 *
 * This file is part of OWL, which extends the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "srslte/phy/ue/ue_dl.h"

#include <complex.h>
#include <math.h>
#include <string.h>


#define CURRENT_FFTSIZE   srslte_symbol_sz(q->cell.nof_prb)
#define CURRENT_SFLEN     SRSLTE_SF_LEN(CURRENT_FFTSIZE)

#define CURRENT_SLOTLEN_RE SRSLTE_SLOT_LEN_RE(q->cell.nof_prb, q->cell.cp)
#define CURRENT_SFLEN_RE SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)

#define RNTILIFELEN 2

static srslte_dci_format_t ue_formats[] = {SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1}; // Only TM1 and TM2 are currently supported 
const uint32_t nof_ue_formats = 2; 

static srslte_dci_format_t common_formats[] = {SRSLTE_DCI_FORMAT1A,SRSLTE_DCI_FORMAT1C};
const uint32_t nof_common_formats = 2; 

//static srslte_dci_format_t ue_all_formats[] = {  SRSLTE_DCI_FORMAT0, SRSLTE_DCI_FORMAT1, SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1B, SRSLTE_DCI_FORMAT1C, SRSLTE_DCI_FORMAT1D, SRSLTE_DCI_FORMAT2, SRSLTE_DCI_FORMAT2A};
//const uint32_t nof_ue_all_formats = 8;

static srslte_dci_format_t ue_all_formats[] = {  SRSLTE_DCI_FORMAT0, SRSLTE_DCI_FORMAT1, SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1B, SRSLTE_DCI_FORMAT1C, SRSLTE_DCI_FORMAT2, SRSLTE_DCI_FORMAT2A};
const uint32_t nof_ue_all_formats = 7;

//srslte_dci_format_t ue_all_formats[] = {  SRSLTE_DCI_FORMAT0, SRSLTE_DCI_FORMAT1A, SRSLTE_DCI_FORMAT1C, SRSLTE_DCI_FORMAT2A};
//const uint32_t nof_ue_all_formats = 4;

int srslte_ue_dl_init(srslte_ue_dl_t *q, 
                      srslte_cell_t cell) 
{
  return srslte_ue_dl_init_multi(q, cell, 1);
}

int srslte_ue_dl_init_multi(srslte_ue_dl_t *q, 
                            srslte_cell_t cell, 
                            uint32_t nof_rx_antennas) 
{
  int ret = SRSLTE_ERROR_INVALID_INPUTS; 
  
  if (q               != NULL             &&
      nof_rx_antennas <= SRSLTE_MAX_PORTS &&
      srslte_cell_isvalid(&cell))   
  {
    ret = SRSLTE_ERROR;
    
    bzero(q, sizeof(srslte_ue_dl_t));
    
    q->cell = cell; 
    q->pkt_errors = 0;
    q->pkts_total = 0;
    q->pending_ul_dci_rnti = 0; 
    q->sample_offset = 0; 
    q->nof_rx_antennas = nof_rx_antennas; 
    
    if (srslte_ofdm_rx_init(&q->fft, q->cell.cp, q->cell.nof_prb)) {
      fprintf(stderr, "Error initiating FFT\n");
      goto clean_exit;
    }
    if (srslte_chest_dl_init(&q->chest, cell)) {
      fprintf(stderr, "Error initiating channel estimator\n");
      goto clean_exit;
    }
    if (srslte_regs_init(&q->regs, q->cell)) {
      fprintf(stderr, "Error initiating REGs\n");
      goto clean_exit;
    }
    if (srslte_pcfich_init_multi(&q->pcfich, &q->regs, q->cell, nof_rx_antennas)) {
      fprintf(stderr, "Error creating PCFICH object\n");
      goto clean_exit;
    }
    if (srslte_phich_init(&q->phich, &q->regs, q->cell)) {
      fprintf(stderr, "Error creating PHICH object\n");
      goto clean_exit;
    }

    if (srslte_pdcch_init_multi(&q->pdcch, &q->regs, q->cell, nof_rx_antennas)) {
      fprintf(stderr, "Error creating PDCCH object\n");
      goto clean_exit;
    }

    if (srslte_pdsch_init_multi(&q->pdsch, q->cell, nof_rx_antennas)) {
      fprintf(stderr, "Error creating PDSCH object\n");
      goto clean_exit;
    }
    if (srslte_softbuffer_rx_init(&q->softbuffer, q->cell.nof_prb)) {
      fprintf(stderr, "Error initiating soft buffer\n");
      goto clean_exit;
    }
    if (srslte_cfo_init(&q->sfo_correct, q->cell.nof_prb*SRSLTE_NRE)) {
      fprintf(stderr, "Error initiating SFO correct\n");
      goto clean_exit;
    }
    srslte_cfo_set_tol(&q->sfo_correct, 1e-5/q->fft.symbol_sz);
    
    for (int j=0;j<nof_rx_antennas;j++) {
      q->sf_symbols_m[j] = srslte_vec_malloc(CURRENT_SFLEN_RE * sizeof(cf_t));
      if (!q->sf_symbols_m[j]) {
        perror("malloc");
        goto clean_exit; 
      }
      for (uint32_t i=0;i<q->cell.nof_ports;i++) {
        q->ce_m[i][j] = srslte_vec_malloc(CURRENT_SFLEN_RE * sizeof(cf_t));
        if (!q->ce_m[i][j]) {
          perror("malloc");
          goto clean_exit; 
        }
      }
    }
    
    srslte_ue_dl_reset_rnti_list(q);

    q->sf_symbols = q->sf_symbols_m[0];
    for (int i=0;i<q->cell.nof_ports;i++) {
      q->ce[i] = q->ce_m[i][0];
    }
    
    ret = SRSLTE_SUCCESS;
  } else {
    fprintf(stderr, "Invalid cell properties: Id=%d, Ports=%d, PRBs=%d\n",
            cell.id, cell.nof_ports, cell.nof_prb);      
  }

clean_exit: 
  if (ret == SRSLTE_ERROR) {
    srslte_ue_dl_free(q);
  }
  return ret;
}

void srslte_ue_dl_free(srslte_ue_dl_t *q) {
  if (q) {
    srslte_ofdm_rx_free(&q->fft);
    srslte_chest_dl_free(&q->chest);
    srslte_regs_free(&q->regs);
    srslte_pcfich_free(&q->pcfich);
    srslte_phich_free(&q->phich);
    srslte_pdcch_free(&q->pdcch);
    srslte_pdsch_free(&q->pdsch);
    srslte_cfo_free(&q->sfo_correct);
    srslte_softbuffer_rx_free(&q->softbuffer);
    for (int j=0;j<q->nof_rx_antennas;j++) {
      if (q->sf_symbols_m[j]) {
        free(q->sf_symbols_m[j]);
      }
      for (uint32_t i=0;i<q->cell.nof_ports;i++) {
        if (q->ce_m[i][j]) {
          free(q->ce_m[i][j]);
        }
      }
    }
    bzero(q, sizeof(srslte_ue_dl_t));
  }
}

void srslte_ue_dl_reset_rnti_list(srslte_ue_dl_t *q) {
	bzero(q->rnti_list, 65536*sizeof(uint8_t));
	bzero(q->rnti_cnt, 65536*sizeof(uint8_t));
	for (int i = 1; i <= 10; i++) {
		q->rnti_list[i] = 1;
	}
	q->rnti_list[65534] = 1;
	q->rnti_list[65535] = 1;
}

void srslte_ue_dl_reset_rnti_user(srslte_ue_dl_t *q, uint16_t user) {
	q->rnti_list[user] = RNTILIFELEN;
	q->rnti_cnt[user]++;
}

void srslte_ue_dl_reset_rnti_user_to(srslte_ue_dl_t *q, uint16_t user, uint16_t val) {
	q->rnti_list[user] = val;
}

void srslte_ue_dl_update_rnti_list(srslte_ue_dl_t *q) {
	for (int i = 10; i < 65533; i++) {
		q->rnti_list[i] = (q->rnti_list[i]>0) ? q->rnti_list[i]-1 : 0;
	}
}

int rnti_in_list(srslte_ue_dl_t *q, uint16_t check) {
	return (q->rnti_list[check]);
}

/* Precalculate the PDSCH scramble sequences for a given RNTI. This function takes a while 
 * to execute, so shall be called once the final C-RNTI has been allocated for the session.
 * For the connection procedure, use srslte_pusch_encode_rnti() or srslte_pusch_decode_rnti() functions 
 */
void srslte_ue_dl_set_rnti(srslte_ue_dl_t *q, uint16_t rnti) {
  srslte_pdsch_set_rnti(&q->pdsch, rnti);
  
  // Compute UE-specific and Common search space for this RNTI
  for (int cfi=0;cfi<3;cfi++) {
    for (int sf_idx=0;sf_idx<10;sf_idx++) {
      q->current_ss_ue[cfi][sf_idx].nof_locations = srslte_pdcch_ue_locations(&q->pdcch, q->current_ss_ue[cfi][sf_idx].loc, MAX_CANDIDATES_UE, sf_idx, cfi+1, rnti);              
    }
    q->current_ss_common[cfi].nof_locations = srslte_pdcch_common_locations(&q->pdcch, q->current_ss_common[cfi].loc, MAX_CANDIDATES_COM, cfi+1);
  }  
  
  q->current_rnti = rnti; 
}

void srslte_ue_dl_reset(srslte_ue_dl_t *q) {
  srslte_softbuffer_rx_reset(&q->softbuffer);
  bzero(&q->pdsch_cfg, sizeof(srslte_pdsch_cfg_t));
}

void srslte_ue_dl_set_sample_offset(srslte_ue_dl_t * q, float sample_offset) {
  q->sample_offset = sample_offset; 
}

/** Applies the following operations to a subframe of synchronized samples: 
 *    - OFDM demodulation
 *    - Channel estimation 
 *    - PCFICH decoding
 *    - PDCCH decoding: Find DCI for RNTI given by previous call to srslte_ue_dl_set_rnti()
 *    - PDSCH decoding: Decode TB scrambling with RNTI given by srslte_ue_dl_set_rnti()
 */
int srslte_ue_dl_decode(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t tti) {
  cf_t *_input[SRSLTE_MAX_PORTS]; 
  _input[0] = input; 
  return srslte_ue_dl_decode_rnti_multi(q, _input, data, tti, q->current_rnti);
}

int srslte_ue_dl_decode_multi(srslte_ue_dl_t *q, cf_t *input[SRSLTE_MAX_PORTS], uint8_t *data, uint32_t tti) {
  return srslte_ue_dl_decode_rnti_multi(q, input, data, tti, q->current_rnti);
}

int srslte_ue_dl_decode_fft_estimate(srslte_ue_dl_t *q, cf_t *input, uint32_t sf_idx, uint32_t *cfi) 
{
  cf_t *_input[SRSLTE_MAX_PORTS]; 
  _input[0] = input; 
  return srslte_ue_dl_decode_fft_estimate_multi(q, _input, sf_idx, cfi);
}

int srslte_ue_dl_decode_fft_estimate_multi(srslte_ue_dl_t *q, cf_t *input[SRSLTE_MAX_PORTS], uint32_t sf_idx, uint32_t *cfi) 
{
  if (input && q && cfi && sf_idx < SRSLTE_NSUBFRAMES_X_FRAME) {
    
    /* Run FFT for all subframe data */
    for (int j=0;j<q->nof_rx_antennas;j++) {
      srslte_ofdm_rx_sf(&q->fft, input[j], q->sf_symbols_m[j]);

      /* Correct SFO multiplying by complex exponential in the time domain */
      if (q->sample_offset) {
        for (int i=0;i<2*SRSLTE_CP_NSYMB(q->cell.cp);i++) {
          srslte_cfo_correct(&q->sfo_correct, 
                          &q->sf_symbols_m[j][i*q->cell.nof_prb*SRSLTE_NRE], 
                          &q->sf_symbols_m[j][i*q->cell.nof_prb*SRSLTE_NRE], 
                          q->sample_offset / q->fft.symbol_sz);
        }
      }
    }
    return srslte_ue_dl_decode_estimate(q, sf_idx, cfi); 
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS; 
  }
}

int srslte_ue_dl_decode_estimate(srslte_ue_dl_t *q, uint32_t sf_idx, uint32_t *cfi) {
  float cfi_corr; 
  if (q && cfi && sf_idx < SRSLTE_NSUBFRAMES_X_FRAME) {
    
    /* Get channel estimates for each port */
    srslte_chest_dl_estimate_multi(&q->chest, q->sf_symbols_m, q->ce_m, sf_idx, q->nof_rx_antennas);

    /* First decode PCFICH and obtain CFI */
    if (srslte_pcfich_decode_multi(&q->pcfich, q->sf_symbols_m, q->ce_m, 
                             srslte_chest_dl_get_noise_estimate(&q->chest), 
                             sf_idx, cfi, &cfi_corr)<0) {
      fprintf(stderr, "Error decoding PCFICH\n");
      return SRSLTE_ERROR;
    }

    INFO("Decoded CFI=%d with correlation %.2f, sf_idx=%d\n", *cfi, cfi_corr, sf_idx);

    if (srslte_regs_set_cfi(&q->regs, *cfi)) {
      fprintf(stderr, "Error setting CFI\n");
      return SRSLTE_ERROR;
    }
    
    return SRSLTE_SUCCESS; 
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS; 
  }
}


int srslte_ue_dl_cfg_grant(srslte_ue_dl_t *q, srslte_ra_dl_grant_t *grant, uint32_t cfi, uint32_t sf_idx, uint32_t rvidx) 
{
  return srslte_pdsch_cfg(&q->pdsch_cfg, q->cell, grant, cfi, sf_idx, rvidx);
}

int srslte_ue_dl_decode_rnti(srslte_ue_dl_t *q, cf_t *input, uint8_t *data, uint32_t tti, uint16_t rnti) 
{
  cf_t *_input[SRSLTE_MAX_PORTS];
  _input[0] = input; 
  return srslte_ue_dl_decode_rnti_multi(q, _input, data, tti, rnti);
}

int srslte_ue_dl_decode_rnti_multi(srslte_ue_dl_t *q, cf_t *input[SRSLTE_MAX_PORTS], uint8_t *data, uint32_t tti, uint16_t rnti) 
{
  srslte_dci_msg_t dci_msg;
  srslte_ra_dl_dci_t dci_unpacked;
  srslte_ra_dl_grant_t grant; 
  int ret = SRSLTE_ERROR; 
  uint32_t cfi;
  
  uint32_t sf_idx = tti%10; 
  
  if ((ret = srslte_ue_dl_decode_fft_estimate_multi(q, input, sf_idx, &cfi)) < 0) {
    return ret; 
  }
  
  float noise_estimate = srslte_chest_dl_get_noise_estimate(&q->chest);
  // Uncoment next line to do ZF by default in pdsch_ue example
  //float noise_estimate = 0; 

  if (srslte_pdcch_extract_llr_multi(&q->pdcch, q->sf_symbols_m, q->ce_m, noise_estimate, sf_idx, cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

  int found_dci = srslte_ue_dl_find_dl_dci(q, cfi, sf_idx, rnti, &dci_msg);   
  if (found_dci == 1) {
    
    if (srslte_dci_msg_to_dl_grant(&dci_msg, rnti, q->cell.nof_prb, q->cell.nof_ports, &dci_unpacked, &grant)) {
      fprintf(stderr, "Error unpacking DCI\n");
      return SRSLTE_ERROR;   
    }
    
    /* ===== These lines of code are supposed to be MAC functionality === */

    
    uint32_t rvidx = 0; 
    if (dci_unpacked.rv_idx < 0) {
      uint32_t sfn = tti/10; 
      uint32_t k   = (sfn/2)%4; 
      rvidx        = ((uint32_t) ceilf((float)1.5*k))%4;
      srslte_softbuffer_rx_reset_tbs(&q->softbuffer, grant.mcs.tbs);      
    } else {
      rvidx = dci_unpacked.rv_idx;
      srslte_softbuffer_rx_reset_tbs(&q->softbuffer, grant.mcs.tbs);      
    }

    if (srslte_ue_dl_cfg_grant(q, &grant, cfi, sf_idx, rvidx)) {
      return SRSLTE_ERROR; 
    }
    
    /* ===== End of MAC functionality ========== */

    q->nof_detected++;
  
    
    if (q->pdsch_cfg.grant.mcs.mod > 0 && q->pdsch_cfg.grant.mcs.tbs >= 0) {
      ret = srslte_pdsch_decode_multi(&q->pdsch, &q->pdsch_cfg, &q->softbuffer, 
                                    q->sf_symbols_m, q->ce_m, 
                                    noise_estimate, 
                                    rnti, data);
      
      if (ret == SRSLTE_ERROR) {
        q->pkt_errors++;
      } else if (ret == SRSLTE_ERROR_INVALID_INPUTS) {
        fprintf(stderr, "Error calling srslte_pdsch_decode()\n");      
      } 
    }
  
  /*
    printf("Saving signal...\n");
    srslte_vec_save_file("input", input, sizeof(cf_t)*SRSLTE_SF_LEN_PRB(q->cell.nof_prb));
    srslte_ue_dl_save_signal(q, &q->softbuffer, sf_idx, rvidx, rnti, cfi);
    //exit(-1);
  */
  
  }

  q->pkts_total++;

  if (found_dci == 1 && ret == SRSLTE_SUCCESS) { 
    return q->pdsch_cfg.grant.mcs.tbs;    
  } else {
    return 0;
  }
}

uint32_t srslte_ue_dl_get_ncce(srslte_ue_dl_t *q) {
  return q->last_location.ncce; 
}

static int dci_blind_search(srslte_ue_dl_t *q, dci_blind_search_t *search_space, uint16_t rnti, srslte_dci_msg_t *dci_msg) 
{
  int ret = SRSLTE_ERROR; 
  uint16_t crc_rem = 0; 
  if (rnti) {
    ret = 0; 
    int i=0;
    while (!ret && i < search_space->nof_locations) {
      DEBUG("Searching format %s in %d,%d (%d/%d)\n", 
             srslte_dci_format_string(search_space->format), search_space->loc[i].ncce, search_space->loc[i].L, 
             i, search_space->nof_locations);
      
      if (srslte_pdcch_decode_msg(&q->pdcch, dci_msg, &search_space->loc[i], search_space->format, &crc_rem)) {
        fprintf(stderr, "Error decoding DCI msg\n");
        return SRSLTE_ERROR;
      }
      if (crc_rem == rnti) {        
        // If searching for Format1A but found Format0 save it for later 
        if (dci_msg->format == SRSLTE_DCI_FORMAT0 && search_space->format == SRSLTE_DCI_FORMAT1A) 
        {
          if (!q->pending_ul_dci_rnti) {
            q->pending_ul_dci_rnti = crc_rem; 
            memcpy(&q->pending_ul_dci_msg, dci_msg, sizeof(srslte_dci_msg_t));          
            memcpy(&q->last_location_ul, &search_space->loc[i], sizeof(srslte_dci_location_t));          
          }
        // Else if we found it, save location and leave
        } else if (dci_msg->format == search_space->format) {
          ret = 1; 
          if (dci_msg->format == SRSLTE_DCI_FORMAT0) {
            memcpy(&q->last_location_ul, &search_space->loc[i], sizeof(srslte_dci_location_t));          
          } else {
            memcpy(&q->last_location, &search_space->loc[i], sizeof(srslte_dci_location_t));          
          }
        } 
      }
      i++; 
    }    
  } else {
    fprintf(stderr, "RNTI not specified\n");
  }
  return ret; 
}

int srslte_ue_dl_find_ul_dci(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  if (rnti && cfi > 0 && cfi < 4) {
    /* Do not search if an UL DCI is already pending */    
    if (q->pending_ul_dci_rnti == rnti) {
      q->pending_ul_dci_rnti = 0;      
      memcpy(dci_msg, &q->pending_ul_dci_msg, sizeof(srslte_dci_msg_t));
      return 1; 
    }
    
    // Configure and run DCI blind search 
    dci_blind_search_t search_space; 
    dci_blind_search_t *current_ss = &search_space;
    if (q->current_rnti == rnti) {
      current_ss = &q->current_ss_ue[cfi-1][sf_idx];
    } else {
      // If locations are not pre-generated, generate them now
      current_ss->nof_locations = srslte_pdcch_ue_locations(&q->pdcch, current_ss->loc, MAX_CANDIDATES_UE, sf_idx, cfi, rnti);        
    }
    
    srslte_pdcch_set_cfi(&q->pdcch, cfi);
    
    current_ss->format = SRSLTE_DCI_FORMAT0; 
    INFO("Searching UL C-RNTI in %d ue locations\n", search_space.nof_locations);
    return dci_blind_search(q, current_ss, rnti, dci_msg);
  } else {
    return 0; 
  }
}

int srslte_ue_dl_find_dl_dci(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  srslte_rnti_type_t rnti_type; 
  if (rnti == SRSLTE_SIRNTI) {
    rnti_type = SRSLTE_RNTI_SI;
  } else if (rnti == SRSLTE_PRNTI) {
    rnti_type = SRSLTE_RNTI_PCH;    
  } else if (rnti <= SRSLTE_RARNTI_END) {
    rnti_type = SRSLTE_RNTI_RAR;    
  } else {
    rnti_type = SRSLTE_RNTI_USER;
  }
  return srslte_ue_dl_find_dl_dci_type(q, cfi, sf_idx, rnti, rnti_type, dci_msg);
}

// Blind search for SI/P/RA-RNTI
static int find_dl_dci_type_siprarnti(srslte_ue_dl_t *q, uint32_t cfi, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  int ret = 0; 
  // Configure and run DCI blind search 
  dci_blind_search_t search_space; 
  search_space.nof_locations = srslte_pdcch_common_locations(&q->pdcch, search_space.loc, MAX_CANDIDATES_COM, cfi);
  INFO("Searching SI/P/RA-RNTI in %d common locations, %d formats\n", search_space.nof_locations, nof_common_formats);
  // Search for RNTI only if there is room for the common search space 
  if (search_space.nof_locations > 0) {    
    for (int f=0;f<nof_common_formats;f++) {
      search_space.format = common_formats[f];   
      if ((ret = dci_blind_search(q, &search_space, rnti, dci_msg))) {
        return ret; 
      }
    }
  }
  return SRSLTE_SUCCESS;   
}

// Blind search for C-RNTI
static int find_dl_dci_type_crnti(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, uint16_t rnti, srslte_dci_msg_t *dci_msg)
{
  int ret = SRSLTE_SUCCESS; 
  dci_blind_search_t search_space; 
  dci_blind_search_t *current_ss = &search_space;
  
  // Search UE-specific search space 
  if (q->current_rnti == rnti) {
    current_ss = &q->current_ss_ue[cfi-1][sf_idx];
  } else {
    // If locations are not pre-generated, generate them now
    current_ss->nof_locations = srslte_pdcch_ue_locations(&q->pdcch, current_ss->loc, MAX_CANDIDATES_UE, sf_idx, cfi, rnti);        
  }
  
  srslte_pdcch_set_cfi(&q->pdcch, cfi);
  
  INFO("Searching DL C-RNTI in %d ue locations, %d formats\n", current_ss->nof_locations, nof_ue_formats);
  for (int f=0;f<nof_ue_formats;f++) {
    current_ss->format = ue_formats[f];   
    if ((ret = dci_blind_search(q, current_ss, rnti, dci_msg))) {
      return ret; 
    }
  }
  
  // Search Format 1A in the Common SS also
  if (q->current_rnti == rnti) {
    current_ss = &q->current_ss_common[cfi-1];
  } else {
    // If locations are not pre-generated, generate them now
    current_ss->nof_locations = srslte_pdcch_common_locations(&q->pdcch, current_ss->loc, MAX_CANDIDATES_COM, cfi);
  }
  
  srslte_pdcch_set_cfi(&q->pdcch, cfi);
  
  // Search for RNTI only if there is room for the common search space 
  if (current_ss->nof_locations > 0) {    
    current_ss->format = SRSLTE_DCI_FORMAT1A; 
    INFO("Searching DL C-RNTI in %d ue locations, format 1A\n", current_ss->nof_locations);
    return dci_blind_search(q, current_ss, rnti, dci_msg);   
  }
  return SRSLTE_SUCCESS; 
}

//srslte_ue_dl_find_dci_cc doesn't have prototype in the ue_dl.h file
int srslte_ue_dl_find_dci_cc(srslte_ue_dl_t *q, srslte_dci_msg_t *dci_msg, uint32_t cfi, uint32_t sf_idx,
			     srslte_rnti_type_t rnti_type, uint32_t sfn,
			     uint16_t *RNTI_array, uint16_t *RNTI_i, uint64_t *dl_bit_sum, uint64_t *ul_bit_sum, uint64_t *dl_rb_sum, uint64_t *ul_rb_sum, FILE *write_fp)
	/* IMDEA contribution: DCI power analysis */
{
  srslte_dci_location_t locations[MAX_CANDIDATES_BLIND];
  uint32_t nof_locations;
  uint32_t nof_formats;
  srslte_ra_dl_dci_t dl_dci_unpacked;
  srslte_ra_ul_dci_t ul_dci_unpacked;
  srslte_ra_dl_grant_t dl_grant;
  srslte_ra_ul_grant_t ul_grant;
  srslte_dci_format_t *formats = NULL;
  uint16_t crc_rem = 0;
  int lprob[MAX_CANDIDATES_BLIND];
  float power = 0;
  int ret = 0;

  /* Generate PDCCH candidates allowing all possible control channel locations in which something is sensed */
  nof_locations = srslte_pdcch_ue_locations_all(&q->pdcch, locations, MAX_CANDIDATES_BLIND, sf_idx, q->cfi);
  formats = ue_all_formats;
  /* Define power only to have a check of all possible size of DCI messages */
#ifdef POWER_ONLY
  nof_formats = 1;
#else
  nof_formats = nof_ue_all_formats;
#endif
  q->current_rnti = 0xffff;
  q->totRBup = 0;
  q->totRBdw = 0;
  q->totBWup = 0;
  q->totBWdw = 0;

  for (int i=0;i<nof_locations;i++) {
	  /* Avoid to check any location whose power is lower than a given threshold and those that have already been checked */
	  if (locations[i].power < PWR_THR || locations[i].checked) {
		  continue;
	  }
	  for (int f=0;f<nof_formats;f++) {
#ifdef POWER_ONLY
		  /* tries all possible DCI sizes and returns the one with the highest re-encoding prob (or the first with a known C-RNTI) */
		  lprob[i] = (int) round(srslte_pdcch_decode_msg_check_power(&q->pdcch, dci_msg, &locations[i], formats[f], &crc_rem, q->rnti_list));
#else
		  /* tries to decode a DCI message of a given format */
		  lprob[i] = (int) round(srslte_pdcch_decode_msg_check(&q->pdcch, dci_msg, &locations[i], formats[f], &crc_rem, q->rnti_list));
#endif
		  /* checks whether the first bit is coherent with the format */
		  if ((formats[f]==SRSLTE_DCI_FORMAT0 && dci_msg->data[0]==1) || (formats[f]==SRSLTE_DCI_FORMAT1A && dci_msg->data[0]==0)) continue;
		  /* checks whether P/SI/RA-RNTI addresses are used with formats other than 0 and 1a */
		  if (!(formats[f]==SRSLTE_DCI_FORMAT0 || formats[f]==SRSLTE_DCI_FORMAT1A) && (crc_rem <= 0x000a || crc_rem > 0xfff3)) continue;
		  /* checks whether the C-RNTI found is coherent with a scheduling in that location */
		  if (!srslte_pdcch_ue_locations_ncce_check((&q->pdcch)->nof_cce, sf_idx, crc_rem, locations[i].ncce)) continue;
		  if (rnti_in_list(q, crc_rem) || lprob[i] >= PROB_THR) {
			  //printf("t=%d.%d (%d (%d)), ret %d, crnti 0x%x\n", sfn, sf_idx, locations[i].ncce, locations[i].L, lprob[i], crc_rem);
			  /* print the DCI information */
			  /* TODO save this information somewhere to be reused by other modules */
		    //MAX NOTE: THIS IS WHERE I REPLACED srslte_dci_msg_to_trace with my own averaging function.
			  if (srslte_dci_msg_to_average(dci_msg, crc_rem, q->cell.nof_prb, q->cell.nof_ports,
				  &dl_dci_unpacked, &ul_dci_unpacked, &dl_grant, &ul_grant, sf_idx, sfn, lprob[i],
							locations[i].ncce, locations[i].L, formats[f], q->cfi, power,
							RNTI_array, RNTI_i, dl_bit_sum, ul_bit_sum, dl_rb_sum, ul_rb_sum,
							write_fp)) {
			    continue;
				  //fprintf(stderr,"1 Error unpacking DCI\n");
			  }
			    
			    /* add the C-RNTI to the white list */
			  srslte_ue_dl_reset_rnti_user(q, crc_rem);
			  ret++;
			  /* checks whether the DCI carries an RA-RNTI. If so marks the current_rnti as the RA-RNTI */
			  if (crc_rem > 0x0000 && crc_rem <= 0x000a && formats[f]!=SRSLTE_DCI_FORMAT0) q->current_rnti = crc_rem;
			  /* save temp information for the graphic version */
			  if (dl_grant.mcs.tbs>0) {
				  q->totRBdw += dl_grant.nof_prb;
				  q->totBWdw += dl_grant.mcs.tbs + dl_grant.mcs2.tbs;
				  if (q->totRBdw > q->cell.nof_prb) q->totBWdw = q->cell.nof_prb;
			  }
			  if (ul_grant.mcs.tbs>0) {
				  q->totRBup += ul_grant.L_prb;
				  q->totBWup += ul_grant.mcs.tbs;
				  if (q->totRBup > q->cell.nof_prb) q->totBWup = q->cell.nof_prb;
			  }
			  /* marks all locations overlapped by and overlapping the current DCI message as already checked */
			  for (int j=i;j<nof_locations;j++) {
				  if (locations[j].ncce >= locations[i].ncce && locations[j].ncce < (locations[i].ncce + (1 << locations[i].L))) {
					  locations[j].checked = 1;
					  DEBUG("skipping location %d with agg %d\n",locations[j].ncce,locations[j].L);
				  }
			  }
			  for (int j=0;j<i;j++) {
				  if (locations[i].ncce >= locations[j].ncce && locations[i].ncce < (locations[j].ncce + (1 << locations[j].L))) {
					  locations[j].checked = 1;
				  }
			  }


		  }
	  } // end of format cycle
  } // end of location cycle
  /* The following print on stderr all locations that possibly contain a DCI and could not be decoded */
#ifdef LOG_ERRORS
  for (int i=0;i<nof_locations;i++) {
	  if (locations[i].power >= PWR_THR && !locations[i].checked) fprintf(stderr,"%d\t%d\t%d\t%d\t%d\n",sfn,sf_idx,locations[i].ncce,locations[i].L,q->cfi);
  }
#endif
  return ret;
}

// MAXNOTE: srslte_ue_dl_get_control_cc is OWL's addition
int srslte_ue_dl_get_control_cc(srslte_ue_dl_t *q, cf_t *input[SRSLTE_MAX_PORTS], uint8_t *data, uint32_t sf_idx, uint32_t rvidx, uint32_t sfn,
				uint16_t *RNTI_array, uint16_t *RNTI_i, uint64_t *dl_bit_sum, uint64_t *ul_bit_sum, uint64_t *dl_rb_sum, uint64_t *ul_rb_sum, FILE * write_fp)
{
  srslte_dci_msg_t dci_msg;
  int ret = SRSLTE_ERROR;


  if ((ret = srslte_ue_dl_decode_fft_estimate_multi(q, input, sf_idx, &q->cfi)) < 0) {
    return ret;
  }

  float noise_estimate = srslte_chest_dl_get_noise_estimate(&q->chest);

  if (srslte_pdcch_extract_llr_multi(&q->pdcch, q->sf_symbols_m, q->ce_m, noise_estimate, sf_idx, q->cfi)) {
    fprintf(stderr, "Error extracting LLRs\n");
    return SRSLTE_ERROR;
  }

  //below contains the prints
  return srslte_ue_dl_find_dci_cc(q, &dci_msg, q->cfi, sf_idx, SRSLTE_RNTI_USER, sfn,
				  RNTI_array, RNTI_i, dl_bit_sum, ul_bit_sum, dl_rb_sum, ul_rb_sum, write_fp);
}

int srslte_ue_dl_find_dl_dci_type(srslte_ue_dl_t *q, uint32_t cfi, uint32_t sf_idx, 
                                  uint16_t rnti, srslte_rnti_type_t rnti_type, srslte_dci_msg_t *dci_msg)
{  
  if (rnti_type == SRSLTE_RNTI_SI || rnti_type == SRSLTE_RNTI_PCH || rnti_type == SRSLTE_RNTI_RAR) {
    return find_dl_dci_type_siprarnti(q, cfi, rnti, dci_msg);
  } else {
    return find_dl_dci_type_crnti(q, cfi, sf_idx, rnti, dci_msg);
  }
}

bool srslte_ue_dl_decode_phich(srslte_ue_dl_t *q, uint32_t sf_idx, uint32_t n_prb_lowest, uint32_t n_dmrs)
{
  uint8_t ack_bit; 
  float distance;
  uint32_t ngroup, nseq; 
  srslte_phich_calc(&q->phich, n_prb_lowest, n_dmrs, &ngroup, &nseq);
  INFO("Decoding PHICH sf_idx=%d, n_prb_lowest=%d, n_dmrs=%d, n_group=%d, n_seq=%d, Ngroups=%d, Nsf=%d\n", 
    sf_idx, n_prb_lowest, n_dmrs, ngroup, nseq, 
    srslte_phich_ngroups(&q->phich), srslte_phich_nsf(&q->phich));
  
  cf_t *ce0[SRSLTE_MAX_PORTS];
  for (int i=0;i<SRSLTE_MAX_PORTS;i++) {
    ce0[i] = q->ce_m[i][0];
  }

  
  if (!srslte_phich_decode(&q->phich, q->sf_symbols_m[0], ce0, 0, ngroup, nseq, sf_idx, &ack_bit, &distance)) {
    INFO("Decoded PHICH %d with distance %f\n", ack_bit, distance);    
  } else {
    fprintf(stderr, "Error decoding PHICH\n");
    return false; 
  }
  if (ack_bit) {
    return true; 
  } else {
    return false; 
  }
}

void srslte_ue_dl_save_signal(srslte_ue_dl_t *q, srslte_softbuffer_rx_t *softbuffer, uint32_t tti, uint32_t rv_idx, uint16_t rnti, uint32_t cfi) {
  srslte_vec_save_file("sf_symbols", q->sf_symbols_m, SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  printf("%d samples\n", SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp));
  srslte_vec_save_file("ce0", q->ce_m[0], SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  if (q->cell.nof_ports > 1) {
    srslte_vec_save_file("ce1", q->ce_m[1], SRSLTE_SF_LEN_RE(q->cell.nof_prb, q->cell.cp)*sizeof(cf_t));
  }
  srslte_vec_save_file("pcfich_ce0", q->pcfich.ce[0], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_ce1", q->pcfich.ce[1], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_symbols", q->pcfich.symbols[0], q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_eq_symbols", q->pcfich.d, q->pcfich.nof_symbols*sizeof(cf_t));
  srslte_vec_save_file("pcfich_llr", q->pcfich.data_f, PCFICH_CFI_LEN*sizeof(float));
  
  srslte_vec_save_file("pdcch_ce0", q->pdcch.ce[0], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_ce1", q->pdcch.ce[1], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_symbols", q->pdcch.symbols[0], q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_eq_symbols", q->pdcch.d, q->pdcch.nof_cce*36*sizeof(cf_t));
  srslte_vec_save_file("pdcch_llr", q->pdcch.llr, q->pdcch.nof_cce*72*sizeof(float));
  
  
  srslte_vec_save_file("pdsch_symbols", q->pdsch.d, q->pdsch_cfg.nbits.nof_re*sizeof(cf_t));
  srslte_vec_save_file("llr", q->pdsch.e, q->pdsch_cfg.nbits.nof_bits*sizeof(cf_t));
  int cb_len = q->pdsch_cfg.cb_segm.K1; 
  for (int i=0;i<q->pdsch_cfg.cb_segm.C;i++) {
    char tmpstr[64]; 
    snprintf(tmpstr,64,"rmout_%d.dat",i);
    srslte_vec_save_file(tmpstr, softbuffer->buffer_f[i], (3*cb_len+12)*sizeof(int16_t));  
  }
  printf("Saved files for tti=%d, sf=%d, cfi=%d, mcs=%d, rv=%d, rnti=0x%x\n", tti, tti%10, cfi, 
         q->pdsch_cfg.grant.mcs.idx, rv_idx, rnti);
}



