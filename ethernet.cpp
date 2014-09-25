#include "config.h"
#include "debug.h"
#include "timing.h"
#include "health.h"
#include "ethernet_phy.h"
#include "mini_ip.h"

const int32_t NTP_FUDGE_RX = (NTP_FUDGE_RX_US * 429497) / 100;
const int32_t NTP_FUDGE_TX = (NTP_FUDGE_TX_US * 429497) / 100;

volatile char ether_int = 0;
uint32_t recv_ts_upper, recv_ts_lower;

static const char ntp_packet_template[48] = {
  4 /* Mode: server reply */ | 3 << 3 /* Version: NTPv3 */,
  1 /* Stratum */, 9 /* Poll: 512sec */, -23 /* Precision: 0.1 usec */,
  0, 0, 0, 0 /* Root delay */,
  0, 0, 0, 10 /* Root Dispersion */,
  'G', 'P', 'S', 0 /* Reference ID */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Reference Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Origin Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Receive Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Transmit Timestamp */
};

void do_ntp_request(unsigned char *pkt, unsigned int len) {
  unsigned char *buf = pkt + 14;
  unsigned char version = (buf[0] >> 3) & 7;
  unsigned char mode = buf[0] & 7;

  if (len < 48) {
    debug("Not NTP\r\n");
    return;
  }

  if (version != 3 && version != 4) {
    debug("NTP unknown version\r\n");
    return;
  }

  if (mode == 3) { /* Client request */
    unsigned char reply[48];
    uint32_t tx_ts_upper, tx_ts_lower, reftime_upper, reftime_lower, rootdisp;

    memcpy(reply, ntp_packet_template, 48);
    /* XXX set Leap Indicator */
    /* Assume a 1ppm error accumulates as long as we're in holdover.
     * This is pessimistic if we were previously locked to Rb. */
    rootdisp = health_get_ref_age() / 15 + 1;
    reply[8] = (rootdisp >> 24) & 0xff;
    reply[9] = (rootdisp >> 16) & 0xff;
    reply[10] = (rootdisp >> 8) & 0xff;
    reply[11] = (rootdisp) & 0xff;

    health_get_reftime(&reftime_upper, &reftime_lower);
    reply[16] = (reftime_upper >> 24) & 0xff;
    reply[17] = (reftime_upper >> 16) & 0xff;
    reply[18] = (reftime_upper >> 8) & 0xff;
    reply[19] = (reftime_upper) & 0xff;
    reply[20] = (reftime_lower >> 24) & 0xff;
    reply[21] = (reftime_lower >> 16) & 0xff;
    reply[22] = (reftime_lower >> 8) & 0xff;
    reply[23] = (reftime_lower) & 0xff;

    /* Copy client transmit timestamp into origin timestamp */
    memcpy(reply + 24, buf + 40, 8);
    /* Copy receive timestamp into packet */
    reply[32] = (recv_ts_upper >> 24) & 0xff;
    reply[33] = (recv_ts_upper >> 16) & 0xff;
    reply[34] = (recv_ts_upper >> 8) & 0xff;
    reply[35] = (recv_ts_upper) & 0xff;
    reply[36] = (recv_ts_lower >> 24) & 0xff;
    reply[37] = (recv_ts_lower >> 16) & 0xff;
    reply[38] = (recv_ts_lower >> 8) & 0xff;
    reply[39] = (recv_ts_lower) & 0xff;

    /* Copy top half of receive timestamp into reference timestamp --
     * we update clock every second :)
     */
    memcpy(reply + 16, reply + 32, 4);

    if (health_get_status() == HEALTH_UNLOCK) {
      reply[1] = 0; /* Stratum: undef */
      memcpy(reply + 12, "INIT", 4); /* refid */
    } else {
      time_get_ntp(*TIMER_CLOCK, &tx_ts_upper, &tx_ts_lower, NTP_FUDGE_TX);
      /* Copy tx timestamp into packet */
      reply[40] = (tx_ts_upper >> 24) & 0xff;
      reply[41] = (tx_ts_upper >> 16) & 0xff;
      reply[42] = (tx_ts_upper >> 8) & 0xff;
      reply[43] = (tx_ts_upper) & 0xff;
      reply[44] = (tx_ts_lower >> 24) & 0xff;
      reply[45] = (tx_ts_lower >> 16) & 0xff;
      reply[46] = (tx_ts_lower >> 8) & 0xff;
      reply[47] = (tx_ts_lower) & 0xff;
    }

    // Send packet
  } else {
    debug("NTP unknown packet type\r\n");
  }
}


unsigned char packet_buffer[256];

void ethernet_pio_setup() {
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB0A_ETXCK,  PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB1A_ETXEN,  PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB2A_ETX0,   PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB3A_ETX1,   PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB4A_ECRSDV, PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB5A_ERX0,   PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB6A_ERX1,   PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB7A_ERXER,  PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB8A_EMDC,   PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB9A_EMDIO,  PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PA5A_TIOA2,  PIO_DEFAULT);
}

static uint8_t gs_uc_mac_address[] = { ETHERNET_MAC_ADDR };
static uint8_t gs_uc_ip_address[] = { ETHERNET_IP_ADDR };

static emac_device_t gs_emac_dev;
static volatile uint8_t gs_uc_eth_buffer[EMAC_FRAME_LENTGH_MAX];

#define SWAP16(n) ( (n & 0xFF) << 8 | (n & 0xFF00) >> 8 )

static uint16_t emac_icmp_checksum(uint16_t *p_buff, uint32_t ul_len)
{
	uint32_t i, ul_tmp;

	for (i = 0, ul_tmp = 0; i < ul_len; i++, p_buff++) {

		ul_tmp += SWAP16(*p_buff);
	}
	ul_tmp = (ul_tmp & 0xffff) + (ul_tmp >> 16);

	return (uint16_t) (~ul_tmp);
}

static void emac_display_ip_packet(p_ip_header_t p_ip_header, uint32_t ul_size)
{
	switch (p_ip_header->ip_p) {
	case IP_PROT_ICMP:
                debug("ICMP");
		break;

	case IP_PROT_IP:
                debug("IP");
		break;

	case IP_PROT_TCP:
                debug("TCP");
		break;

	case IP_PROT_UDP:
                debug("UDP");
		break;

	default:
                debug("Proto "); debug(p_ip_header->ip_p);
		break;
	}

        debug(" packet from ");
        debug(p_ip_header->ip_src[0]); debug(".");
        debug(p_ip_header->ip_src[1]); debug(".");
        debug(p_ip_header->ip_src[2]); debug(".");
        debug(p_ip_header->ip_src[3]); debug("\r\n");
}

static void emac_process_arp_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint32_t i;
	uint8_t ul_rc = EMAC_OK;

	p_ethernet_header_t p_eth = (p_ethernet_header_t) p_uc_data;
	p_arp_header_t p_arp = (p_arp_header_t) (p_uc_data + ETH_HEADER_SIZE);

	if (SWAP16(p_arp->ar_op) == ARP_REQUEST) {
		// ARP reply operation
		p_arp->ar_op = SWAP16(ARP_REPLY);

		// Fill the destination address and source address
		for (i = 0; i < 6; i++) {
			// Swap ethernet destination address and ethernet source address
			p_eth->et_dest[i] = p_eth->et_src[i];
			p_eth->et_src[i] = gs_uc_mac_address[i];
			p_arp->ar_tha[i] = p_arp->ar_sha[i];
			p_arp->ar_sha[i] = gs_uc_mac_address[i];
		}
		// Swap the source IP address and the destination IP address
		for (i = 0; i < 4; i++) {
			p_arp->ar_tpa[i] = p_arp->ar_spa[i];
			p_arp->ar_spa[i] = gs_uc_ip_address[i];
		}
		ul_rc = emac_dev_write(&gs_emac_dev, p_uc_data, ul_size, NULL);
		if (ul_rc != EMAC_OK) {
                        debug("ARP send error: 0x"); debug_hex(ul_rc); debug("\r\n");
		}
	}
}

static void emac_process_ip_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint32_t i;
	uint32_t ul_icmp_len;
	int32_t ul_rc = EMAC_OK;

	ul_size = ul_size;	// stop warning

	p_ethernet_header_t p_eth = (p_ethernet_header_t) p_uc_data;
	p_ip_header_t p_ip_header = (p_ip_header_t) (p_uc_data + ETH_HEADER_SIZE);

	p_icmp_echo_header_t p_icmp_echo =
			(p_icmp_echo_header_t) ((int8_t *) p_ip_header +
			ETH_IP_HEADER_SIZE);
	switch (p_ip_header->ip_p) {
	case IP_PROT_ICMP:
		if (p_icmp_echo->type == ICMP_ECHO_REQUEST) {
			p_icmp_echo->type = ICMP_ECHO_REPLY;
			p_icmp_echo->code = 0;
			p_icmp_echo->cksum = 0;

			// Checksum of the ICMP message
			ul_icmp_len = (SWAP16(p_ip_header->ip_len) - ETH_IP_HEADER_SIZE);
			if (ul_icmp_len % 2) {
				*((uint8_t *) p_icmp_echo + ul_icmp_len) = 0;
				ul_icmp_len++;
			}
			ul_icmp_len = ul_icmp_len / sizeof(uint16_t);

			p_icmp_echo->cksum = SWAP16(
					emac_icmp_checksum((uint16_t *)p_icmp_echo, ul_icmp_len));
			// Swap the IP destination  address and the IP source address
			for (i = 0; i < 4; i++) {
				p_ip_header->ip_dst[i] =
						p_ip_header->ip_src[i];
				p_ip_header->ip_src[i] = gs_uc_ip_address[i];
			}
			// Swap ethernet destination address and ethernet source address
			for (i = 0; i < 6; i++) {
				// Swap ethernet destination address and ethernet source address
				p_eth->et_dest[i] = p_eth->et_src[i];
				p_eth->et_src[i] = gs_uc_mac_address[i];
			}
			// Send the echo_reply
			ul_rc = emac_dev_write(&gs_emac_dev, p_uc_data,
					SWAP16(p_ip_header->ip_len) + 14, NULL);
			if (ul_rc != EMAC_OK) {
                                debug("ICMP send error: 0x"); debug_hex(ul_rc); debug("\r\n");
			}
		}
		break;

	default:
		break;
	}
}

static void emac_process_eth_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint16_t us_pkt_format;

	p_ethernet_header_t p_eth = (p_ethernet_header_t) (p_uc_data);
	p_ip_header_t p_ip_header = (p_ip_header_t) (p_uc_data + ETH_HEADER_SIZE);
	ip_header_t ip_header;
	us_pkt_format = SWAP16(p_eth->et_protlen);

	switch (us_pkt_format) {
	// ARP Packet format
	case ETH_PROT_ARP:
		// Process the ARP packet
		emac_process_arp_packet(p_uc_data, ul_size);

		break;

	// IP protocol frame
	case ETH_PROT_IP:
		// Backup the header
		memcpy(&ip_header, p_ip_header, sizeof(ip_header_t));

		// Process the IP packet
		emac_process_ip_packet(p_uc_data, ul_size);

		// Dump the IP header
		emac_display_ip_packet(&ip_header, ul_size);
		break;

	default:
		break;
	}
}

void EMAC_Handler(void)
{
	debug("EH\r\n");
        ether_int = 1;
	emac_handler(&gs_emac_dev);
}

void ether_init() {
	uint32_t ul_frm_size;
	volatile uint32_t ul_delay;
	emac_options_t emac_option;
   
	ethernet_pio_setup();

 	// Display MAC & IP settings
        debug("MAC: ");
        debug_hex(gs_uc_mac_address[0]); debug(":");
        debug_hex(gs_uc_mac_address[1]); debug(":");
        debug_hex(gs_uc_mac_address[2]); debug(":");
        debug_hex(gs_uc_mac_address[3]); debug(":");
        debug_hex(gs_uc_mac_address[4]); debug(":");
        debug_hex(gs_uc_mac_address[5]); debug("\r\n");

        debug("IP: ");
        debug(gs_uc_ip_address[0]); debug(".");
        debug(gs_uc_ip_address[1]); debug(".");
        debug(gs_uc_ip_address[2]); debug(".");
        debug(gs_uc_ip_address[3]); debug("\r\n");

	// Reset PHY
	rstc_set_external_reset(RSTC, 13); // (2^(13+1))/32768
	rstc_reset_extern(RSTC);
	while (rstc_get_status(RSTC) & RSTC_SR_NRSTL) {
	};

	// Wait for PHY to be ready (CAT811: Max400ms)
	ul_delay = SystemCoreClock / 1000 / 3 * 400;
	while (ul_delay--);

	// Enable EMAC clock
	pmc_enable_periph_clk(ID_EMAC);

	// Fill in EMAC options
	emac_option.uc_copy_all_frame = 0;
	emac_option.uc_no_boardcast = 0;

	memcpy(emac_option.uc_mac_addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));

	gs_emac_dev.p_hw = EMAC;
  
        debug("Init EMAC driver structure\r\n");
	// Init EMAC driver structure
	emac_dev_init(EMAC, &gs_emac_dev, &emac_option);

	// Enable Interrupt
	NVIC_EnableIRQ(EMAC_IRQn);
        
        debug("Init MAC PHY driver\r\n");

	// Init MAC PHY driver
	if (ethernet_phy_init(EMAC, BOARD_EMAC_PHY_ADDR, SystemCoreClock)
					!= EMAC_OK) {
		debug("PHY Initialize ERROR!\r\n");
		//return -1;
	}

	// Auto Negotiate, work in RMII mode
	if (ethernet_phy_auto_negotiate(EMAC, BOARD_EMAC_PHY_ADDR) != EMAC_OK) {

		debug("Auto Negotiate ERROR!\r\n");
		//return -1;
	}
        
        debug("Establish ethernet link... ");
	// Establish ethernet link
	while (ethernet_phy_set_link(EMAC, BOARD_EMAC_PHY_ADDR, 1) != EMAC_OK) {
          debug("ERROR\r\n");
	}
        debug("OK\r\n");
}

void ether_recv() {
	// Process packets
	uint32_t ul_frm_size;
	if (EMAC_OK != emac_dev_read(&gs_emac_dev, (uint8_t *) gs_uc_eth_buffer,
					sizeof(gs_uc_eth_buffer), &ul_frm_size)) {
		return;
	}

	if (ul_frm_size > 0) {
		// Handle input frame
		emac_process_eth_packet((uint8_t *) gs_uc_eth_buffer, ul_frm_size);
	}
}
