// SPDX-License-Identifier: GPL-2.0
/*
 * GXP UMAC driver
 *
 * (C) Copyright 2020 Hewlett Packard Enterprise Development LP.
 *
*/

#define UMAC0_BASE_ADDR  0xc0004000
#define UMAC1_BASE_ADDR  0xc0005000
#define EXTERNAL_MDIO_BASE_ADDR  (UMAC0_BASE_ADDR + 0x80)
#define INTERNAL_MDIO_BASE_ADDR  (UMAC1_BASE_ADDR + 0x80)

#define UMAC_CFG_STAT_1_REG_TXEN_MASK (1 << 12)
#define UMAC_CFG_STAT_1_REG_RXEN_MASK (1 << 11)

#define UMAC_MAC_INT_CFG_REG_TX_INT_MASK 0x0
#define UMAC_MAC_INT_CFG_REG_RX_INT_MASK 0x4
#define UMAC_MAC_INT_CFG_REG_OVERRUN_INT_MASK 0x10
#define UMAC_CFG_STAT_1_REG_MISSED_MASK 0x80
#define UMAC_RING_RX_ERR_MASK       0x38E0

#define UMAC_RING_SIZE_TX_SHIFT 24
#define UMAC_RING_SIZE_RX_SHIFT 16

// maximum Ethernet frame size
#define UMAC_MIN_FRAME_SIZE       60    // excludes preamble, sfd, and fcs
#define UMAC_MAX_FRAME_SIZE       1514  // excludes preamble, sfd, and fcs
#define UMAC_MAX_PACKET_ROUNDED   0x600 // 1536, nicely aligned
#define UMAC_RING_ENTRY_HW_OWN  0x8000

// u-boot is single threaded so more than a single entry really isn't needed.
// the fewest number that can be allocated is 4
// J: Test 256
#define UMAC_MAX_RING_ENTRIES           4

#define GXP_EXTERNAL_MII_NAME	"GXP_MDIO_EXT"
#define GXP_INTERNAL_MII_NAME	"GXP_MDIO_INT"

// register offsets
struct gxp_umac_regs {
	uint32_t config_status;    		//  R/W	Configuration and Status Register I
	uint32_t ring_ptr;         		//  R/W	Ring Pointer Register
	uint32_t ring_prompt;     	 	//  W	Ring Prompt Register
	uint32_t clear_status;     		//  W	Clear Status Register
	uint32_t cksum_config;   	  	//  R/W	Checksum Config Register
	uint32_t ring_size;        		//  R/W	Ring Size Register
	uint32_t mac_addr_hi;      		//  R/W	MAC Address[47:32] Register
	uint32_t mac_addr_mid;     		//  R/W	MAC Address[31:16] Register
	uint32_t mac_addr_lo;      		//  R/W	MAC Address[15:0] Register
	uint32_t mc_addr_filt_hi;  		//  R/W	LAF[63:32] Register
	uint32_t mc_addr_filt_lo;  		//  R/W	LAF[31:0] Register
	uint32_t config_status2;   		//  R/W	Configuration and Status Register II
	uint32_t interrupt;        		//  R/W	MAC Interrupt Configuration and Status Register
	uint32_t overrun_count;    		//  R/W	Overrun Counter Register
	uint32_t rx_int_config;    		//  R/W	Rx Interrupt Config Register
	uint32_t tx_int_config;    		//  R/W	Tx Interrupt Config Register
	uint32_t packet_length;    		//  R/W	Packet Length Register
	uint32_t bcast_filter;     		//  R/W	Broadcast Filter Config Register
	uint32_t bcast_prompt;     		//  W	Broadcast Prompt Register
	uint32_t rx_ring_addr;     		//  R/W	Rx Ring Base Address Register
	uint32_t tx_ring_addr;     		//  R/W	Tx Ring Base Address Register
	uint32_t dma_config;       		//  R/W	DMA Config Register
	uint32_t burst_config;     		//  R/W	Bursting Config Register
	uint32_t pause_config;     		//  R/W	PAUSE Frame Config Register
	uint32_t pause_control;    		//  R/W	PAUSE Frame Control and Status Register
	uint32_t congestion_config; 	//  R/W	Channel Congestion Config Register
	uint32_t frame_filter_config; 	//  R/W	Frame Type Filter Config Register
	uint32_t rx_fifo_config_status; //  R/W	RX FIFO Config and Status register
	uint32_t rx_ring1_base_addr;	//  R/W	RX Ring 1 Base Address Register
	uint32_t config_status3;   		//  R/W	Configuration and Status Register III
	uint32_t unused1;		   		//  spacer
	uint32_t unused2;		   		//  spacer
	uint32_t mmi;              		//  R/W	MMI Register
	uint32_t mmi_data;         		//  R/W	MMI Data Register
	uint32_t link;             		//  R/W	Link Register
	uint32_t mmi_config;       		//  R/W	MMI configuration register
};

#pragma pack(1)

struct umac_packet {
	uint8_t data[UMAC_MAX_FRAME_SIZE];	// actual packet data
	uint8_t pad[UMAC_MAX_PACKET_ROUNDED-UMAC_MAX_FRAME_SIZE];	// pad to make a packet entry nice and aligned
};

struct umac_common_ring_entry {
	uint32_t dma_address;
	uint16_t status;
	uint16_t count;
	uint32_t specialized;
	uint32_t reserved;
};

struct umac_rx_ring_entry {
	uint32_t dma_address;
	uint16_t status;
	uint16_t count;
	uint16_t checksum;
	uint16_t control;
	uint32_t reserved;
};

struct umac_tx_ring_entry {
	uint32_t dma_address;
	uint16_t status;
	uint16_t count;
	uint32_t cksum_offset;
	uint32_t reserved;
};

typedef union {
	struct umac_common_ring_entry common;
	struct umac_rx_ring_entry rx;
	struct umac_tx_ring_entry tx;
} umac_ring_entry;

#pragma pack()

struct umac_ring {
	umac_ring_entry *ring_base;
	umac_ring_entry *producer_ptr;
	umac_ring_entry *completion_ptr;
	uint32_t ring_size_bytes;
};

struct umac_struct {
	uint32_t base_addr;
	uint32_t phy_type;
	uint32_t phy_num;
	uint8_t mac_addr[6];
	struct umac_ring txring;
	struct umac_ring rxring;
	struct umac_packet *rx_packet_buf;
	void *rx_packet_map_base;
	uint32_t rx_packet_map_size;
	uint32_t num_rx_packet_buf_bytes;
};

struct gxp_umac_private {
	struct mii_dev *external_bus;
	int external_dev_num;
	struct mii_dev *internal_bus;
	int internal_dev_num;
	struct phy_device *phydev;
	phy_interface_t interface;
	struct umac_struct umac;
};
