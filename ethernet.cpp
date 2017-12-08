#include "config.h"
#include "debug.h"
#include "timing.h"
#include "health.h"
#include "monitor.h"
#include "ethernet_phy.h"
#include "mini_ip.h"

static uint8_t gs_uc_mac_address[] = { ETHERNET_MAC_ADDR };
static uint8_t gs_uc_ip_address[] = { ETHERNET_IP_ADDR };
static emac_device_t gs_emac_dev;
static volatile uint8_t gs_uc_eth_buffer[EMAC_FRAME_LENTGH_MAX];

#define SWAP16(n) ( (n & 0xFF) << 8 | (n & 0xFF00) >> 8 )

const int32_t NTP_FUDGE_RX = (NTP_FUDGE_RX_US * 429497) / 100;
const int32_t NTP_FUDGE_TX = (NTP_FUDGE_TX_US * 429497) / 100;

volatile char ether_int = 0;
uint32_t eh_ts_upper, eh_ts_lower;
uint32_t recv_ts_upper, recv_ts_lower;

int ntp_invalid = 0, ntp_wrongversion = 0, ntp_wrongmode = 0, ntp_error = 0, ntp_ok = 0;

void (*arp_callback)(unsigned char[], unsigned char[]);

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

void ethernet_send_udp_packet(const char dst_ip[4], const char dst_mac[6],
    uint16_t dst_port, uint16_t src_port, const char *payload, unsigned int len) {
  unsigned char sndbuf[ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE + 1024];
  uint32_t ip_checksum = 0;

  if (len > 1024) {
    debug("Tried to send a too-long packet");
    return;
  }

  p_ethernet_header_t p_eth_header = (p_ethernet_header_t)sndbuf;
  p_ip_header_t p_ip_header = (p_ip_header_t)(sndbuf + ETH_HEADER_SIZE);
  p_udp_header_t p_udp_header = (p_udp_header_t)(sndbuf + ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE);
  unsigned char *payload_out = sndbuf + ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE;

  memcpy(p_eth_header->et_dest, dst_mac, 6);
  memcpy(p_eth_header->et_src, gs_uc_mac_address, 6);
  p_eth_header->et_protlen = SWAP16(0x0800);


  p_ip_header->ip_hl_v = 0x45;
  p_ip_header->ip_tos = 0;
  p_ip_header->ip_len = SWAP16(ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE + len);
  p_ip_header->ip_id = 0;
  p_ip_header->ip_off = 0;
  p_ip_header->ip_ttl = 64;
  p_ip_header->ip_p = IP_PROT_UDP;
  p_ip_header->ip_sum = 0;
  memcpy(p_ip_header->ip_src, gs_uc_ip_address, 4);
  memcpy(p_ip_header->ip_dst, dst_ip, 4);

  for (uint16_t *p = (uint16_t *)p_ip_header ; p < (uint16_t *)p_ip_header + 10 ; p++) {
    ip_checksum += SWAP16(*p);
  }
  while (ip_checksum > 0xffff) {
    ip_checksum = (ip_checksum & 0xffff) + (ip_checksum >> 16);
  }
  p_ip_header->ip_sum = ~SWAP16(ip_checksum);


  p_udp_header->port_src = SWAP16(src_port);
  p_udp_header->port_dst = SWAP16(dst_port);
  p_udp_header->length = SWAP16(ETH_UDP_HEADER_SIZE + len);
  p_udp_header->cksum = 0;

  memcpy(payload_out, payload, len);
  uint8_t ul_rc = emac_dev_write(&gs_emac_dev, sndbuf,
      ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE + len, NULL);
  if (ul_rc != EMAC_OK) {
    debug("UDP send error: 0x"); debug_hex(ul_rc); debug("\r\n");
  }
}


void do_ntp_request(unsigned char *pkt, unsigned int len) {
  p_ethernet_header_t p_eth_header = (p_ethernet_header_t)pkt;
  p_ip_header_t p_ip_header = (p_ip_header_t)(pkt + ETH_HEADER_SIZE);
  p_udp_header_t p_udp_header = (p_udp_header_t)(pkt + ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE);
  unsigned char *buf = pkt + ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE;
  unsigned char version = (buf[0] >> 3) & 7;
  unsigned char mode = buf[0] & 7;

  if (len < 48) {
    debug("Not NTP\r\n");
    ntp_invalid++;
    return;
  }

  if (version != 3 && version != 4) {
    debug("NTP unknown version ");
    debug(version);
    debug("\r\n");
    ntp_wrongversion++;
    return;
  }

  if (mode == 3) { /* Client request */
    // Fill the destination address and source address
    for (int i = 0; i < 6; i++) {
      // Swap ethernet destination address and ethernet source address
      p_eth_header->et_dest[i] = p_eth_header->et_src[i];
      p_eth_header->et_src[i] = gs_uc_mac_address[i];
    }
    // Swap the source IP address and the destination IP address
    for (int i = 0; i < 4; i++) {
      p_ip_header->ip_dst[i] = p_ip_header->ip_src[i];
      p_ip_header->ip_src[i] = gs_uc_ip_address[i];
    }
    // Swap the UDP src and dest port
    p_udp_header->port_dst = p_udp_header->port_src;
    p_udp_header->port_src = SWAP16(123);
    p_udp_header->cksum = 0;

    uint32_t tx_ts_upper, tx_ts_lower, reftime_upper, reftime_lower, rootdisp;
    unsigned char origin_ts[8];
    memcpy(origin_ts, buf + 40, 8);

    memcpy(buf, ntp_packet_template, 48);
    /* XXX set Leap Indicator */
    /* Assume a 1ppm error accumulates as long as we're in holdover.
     * This is pessimistic if we were previously locked to Rb. */
    rootdisp = health_get_ref_age() / 15 + 1;
    buf[8] = (rootdisp >> 24) & 0xff;
    buf[9] = (rootdisp >> 16) & 0xff;
    buf[10] = (rootdisp >> 8) & 0xff;
    buf[11] = (rootdisp) & 0xff;

    health_get_reftime(&reftime_upper, &reftime_lower);
    buf[16] = (reftime_upper >> 24) & 0xff;
    buf[17] = (reftime_upper >> 16) & 0xff;
    buf[18] = (reftime_upper >> 8) & 0xff;
    buf[19] = (reftime_upper) & 0xff;
    buf[20] = (reftime_lower >> 24) & 0xff;
    buf[21] = (reftime_lower >> 16) & 0xff;
    buf[22] = (reftime_lower >> 8) & 0xff;
    buf[23] = (reftime_lower) & 0xff;

    /* Copy client transmit timestamp into origin timestamp */
    memcpy(buf + 24, origin_ts, 8);
    /* Copy receive timestamp into packet */
    buf[32] = (recv_ts_upper >> 24) & 0xff;
    buf[33] = (recv_ts_upper >> 16) & 0xff;
    buf[34] = (recv_ts_upper >> 8) & 0xff;
    buf[35] = (recv_ts_upper) & 0xff;
    buf[36] = (recv_ts_lower >> 24) & 0xff;
    buf[37] = (recv_ts_lower >> 16) & 0xff;
    buf[38] = (recv_ts_lower >> 8) & 0xff;
    buf[39] = (recv_ts_lower) & 0xff;

    if (health_get_status() == HEALTH_UNLOCK) {
      buf[1] = 0; /* Stratum: undef */
      memcpy(buf + 12, "INIT", 4); /* refid */
    } else {
      time_get_ntp(*TIMER_CLOCK, &tx_ts_upper, &tx_ts_lower, NTP_FUDGE_TX);
      /* Copy tx timestamp into packet */
      buf[40] = (tx_ts_upper >> 24) & 0xff;
      buf[41] = (tx_ts_upper >> 16) & 0xff;
      buf[42] = (tx_ts_upper >> 8) & 0xff;
      buf[43] = (tx_ts_upper) & 0xff;
      buf[44] = (tx_ts_lower >> 24) & 0xff;
      buf[45] = (tx_ts_lower >> 16) & 0xff;
      buf[46] = (tx_ts_lower >> 8) & 0xff;
      buf[47] = (tx_ts_lower) & 0xff;
    }

    uint8_t ul_rc = emac_dev_write(&gs_emac_dev, pkt,
        48 + ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE, NULL);
    if (ul_rc != EMAC_OK) {
      debug("NTP send error: 0x"); debug_hex(ul_rc); debug("\r\n");
      ntp_error++;
    } else {
      ntp_ok++;
    }
  } else {
    ntp_wrongmode++;
  }
}

void ethernet_send_ntp_stats() {
  monitor_send("ntp.invalid", ntp_invalid);
  monitor_send("ntp.wrongversion", ntp_wrongversion);
  monitor_send("ntp.wrongmode", ntp_wrongmode);
  monitor_send("ntp.error", ntp_error);
  monitor_send("ntp.ok", ntp_ok);

  ntp_invalid = 0;
  ntp_wrongversion = 0;
  ntp_wrongmode = 0;
  ntp_error = 0;
  ntp_ok = 0;
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
  } else if (SWAP16(p_arp->ar_op) == ARP_REPLY && arp_callback) {
    arp_callback(p_arp->ar_spa, p_arp->ar_sha);
  }
}

static void emac_process_ip_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
  uint32_t i;
  uint32_t ul_icmp_len;
  int32_t ul_rc = EMAC_OK;
  uint16_t dst_port;

  p_ethernet_header_t p_eth = (p_ethernet_header_t) p_uc_data;
  p_ip_header_t p_ip_header = (p_ip_header_t) (p_uc_data + ETH_HEADER_SIZE);

  p_udp_header_t p_udp_header =
    (p_udp_header_t) ((int8_t *) p_ip_header +
        ETH_IP_HEADER_SIZE);
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
    case IP_PROT_UDP:
      dst_port = SWAP16(p_udp_header->port_dst);
      if (dst_port == 123) {
        do_ntp_request(
            p_uc_data,
            ul_size - (ETH_HEADER_SIZE + ETH_IP_HEADER_SIZE + ETH_UDP_HEADER_SIZE)
            );
      } else {
        //			debug("UDP port "); debug(dst_port); debug("\r\n");
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
      break;

    default:
      break;
  }
}

void EMAC_Handler(void)
{
  time_get_ntp(*TIMER_CLOCK, &eh_ts_upper, &eh_ts_lower, NTP_FUDGE_RX);
  emac_handler(&gs_emac_dev);
}

void ether_rx_handler(uint32_t rx_status) {
  recv_ts_upper = eh_ts_upper;
  recv_ts_lower = eh_ts_lower;
  ether_int = 1;
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

  emac_dev_set_rx_callback(&gs_emac_dev, ether_rx_handler);
}

void ether_recv() {
  // Process packets
  uint32_t ul_frm_size;
  while (emac_dev_read(&gs_emac_dev, (uint8_t *) gs_uc_eth_buffer,
        sizeof(gs_uc_eth_buffer), &ul_frm_size) == EMAC_OK) {
    if (ul_frm_size > 0) {
      // Handle input frame
      emac_process_eth_packet((uint8_t *) gs_uc_eth_buffer, ul_frm_size);
    } else {
      break;
    }
  }
  ether_int = 0;
}
