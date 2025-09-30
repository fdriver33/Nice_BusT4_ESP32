/*
  Nice BusT4
  Data exchange via UART at speed 19200 8n1
  Before the data packet a break of 519 µs (10 bits) is sent
  The contents of the packet that were understood are described in the structure packet_cmd_body_t

  For Oview, 0x80 is always added to the address.
  The gate controller address remains unchanged.

Connection

BusT4                       ESP8266

Device wall              Rx Tx GND
9  7  5  3  1
10 8  6  4  2
place for cable
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V

From the manual nice_dmbm_integration_protocol.pdf

• ADR: this is the NICE network address where the devices you want to control are located. It can have a value from 1 to 63 (from 1 to 3F).
This value must be in HEX. If the addressee is the DIN-BAR integration module, this value is 0 (adr = 0). If the addressee
is an intelligent motor, this value is 1 (adr = 1).
• EPT: this is the Nice motor address within the ADR network. It can have a value from 1 to 127. This value must be in HEX.
• CMD: this is the command you want to send to the destination (ADR, EPT).
• PRF: profile setup command.
• FNC: this is the function you want to send to the destination (ADR, EPT).
• EVT: this is the event that is sent to the destination (ADR, EPT).
*/

#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // for adding Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // parse strings with built-in tools
#include <queue>                               // for working with a queue

namespace esphome {
namespace bus_t4 {

/* for shorter access to class members */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;

static const int _UART_NO = UART0;         /* UART number */
static const int TX_P = 1;                 /* Tx pin */
static const uint32_t BAUD_BREAK = 9200;   /* baud rate for the long pulse before a packet */
static const uint32_t BAUD_WORK  = 19200;  /* working baud rate */
static const uint8_t START_CODE  = 0x55;   /* start byte of a packet */

static const float    CLOSED_POSITION_THRESHOLD = 0.007;  // Drive position value (in percent) below which the gate is considered fully closed
static const uint32_t POSITION_UPDATE_INTERVAL  = 500;    // Interval for updating the current drive position, ms

/* ESP network settings
  Series can take values from 0 to 63, default is 0
  OVIEW address starts from 8

  When combining several drives with OXI into one network you need to set different series values for different drives.
  In this case the OXI series must match the drive it controls.
*/

/* Packet message type
  For now we are only interested in CMD and INF
  Others were not deeply studied and numbers were not verified
  6th byte of CMD and INF packets
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* verified number, sending commands to the automation */
//  LSC = 0x02,  /* work with scenario lists */
//  LST = 0x03,  /* work with automation lists */
//  POS = 0x04,  /* request and change automation position */
//  GRP = 0x05,  /* send command to a group of automations with a motor bitmask */
//  SCN = 0x06,  /* work with scenarios */
//  GRC = 0x07,  /* send command to groups created via Nice Screen Configuration Tool */
  INF = 0x08,  /* returns or sets device information */
//  LGR = 0x09,  /* work with group lists */
//  CGR = 0x0A,  /* work with group categories created via Nice Screen Configuration Tool */
};

/*
Command menu in the Oview hierarchy
9th byte of CMD packets
*/
enum cmd_mnu : uint8_t {
  CONTROL = 0x01,
};

/* Used in STA responses */
enum sub_run_cmd2 : uint8_t {
  STA_OPENING  = 0x02,
  STA_CLOSING  = 0x03,
  OPENED       = 0x04,
  CLOSED       = 0x05,
  ENDTIME      = 0x06,  // maneuver finished by timeout
  STOPPED      = 0x08,
  PART_OPENED  = 0x10,  // partial opening
};

/* Errors */
enum errors_byte : uint8_t {
  NOERR = 0x00, // No errors
  FD    = 0xFD, // No command for this device
};

// Motor types
enum motor_type : uint8_t {
  SLIDING   = 0x01,
  SECTIONAL = 0x02,
  SWING     = 0x03,
  BARRIER   = 0x04,
  UPANDOVER = 0x05, // up-and-over lifting-turning gates
};

//  Ninth byte
enum whose_pkt : uint8_t {
  FOR_ALL = 0x00,  /* packet to/from all */
  FOR_CU  = 0x04,  /* packet to/from the control unit */
  FOR_OXI = 0x0A,  /* packet to/from the OXI receiver */
};

// Tenth byte of GET/SET EVT packets; for CMD packets only the RUN value has been seen
enum command_pkt : uint8_t {
  TYPE_M       = 0x00,  /* Drive type request */
  INF_STATUS   = 0x01,  // Gate state (Open/Closed/Stopped)
  WHO          = 0x04,  /* Who is on the network? */
  MAC          = 0x07,  // MAC address
  MAN          = 0x08,  // manufacturer
  PRD          = 0x09,  // product
  INF_SUPPORT  = 0x10,  // Available INF commands
  HWR          = 0x0A,  // hardware version
  FRM          = 0x0B,  // firmware version
  DSC          = 0x0C,  // description
  CUR_POS      = 0x11,  // current conditional position of the automation; DPRO924 expects position values after this
  MAX_OPN      = 0x12,  // Maximum possible opening by encoder
  POS_MAX      = 0x18,  // Maximum position (opening) by encoder
  POS_MIN      = 0x19,  // Minimum position (closing) by encoder
  INF_P_OPN1   = 0x21,  // Partial opening 1
  INF_P_OPN2   = 0x22,  // Partial opening 2
  INF_P_OPN3   = 0x23,  // Partial opening 3
  INF_SLOW_OPN = 0x24,  // Slowdown during opening
  INF_SLOW_CLS = 0x25,  // Slowdown during closing
  OPN_OFFSET   = 0x28,  /* Opening delay */
  CLS_OFFSET   = 0x29,  /* Closing delay */
  OPN_DIS      = 0x2A,  /* Main parameters - Opening discharge */
  CLS_DIS      = 0x2B,  /* Main parameters - Closing discharge */
  REV_TIME     = 0x31,  /* Main parameters - Reverse duration (Brief inversion value) */
  OPN_PWR      = 0x4A,  /* Main parameters - Force control - Opening force */
  CLS_PWR      = 0x4B,  /* Main parameters - Force control - Closing force */
  SPEED_OPN    = 0x42,  /* Main parameters - Speed setup - Opening speed */
  SPEED_CLS    = 0x43,  /* Main parameters - Speed setup - Closing speed */
  SPEED_SLW_OPN= 0x45,  /* Main parameters - Speed setup - Slow opening speed */
  SPEED_SLW_CLS= 0x46,  /* Main parameters - Speed setup - Slow closing speed */
  OUT1         = 0x51,  /* Output setup */
  OUT2         = 0x52,  /* Output setup */
  LOCK_TIME    = 0x5A,  /* Output setup - Lock operation time */
  S_CUP_TIME   = 0x5C,  /* Output setup - Suction Cup operation time */
  LAMP_TIME    = 0x5B,  /* Output setup - Courtesy light time */
  COMM_SBS     = 0x61,  /* Command setup - Step by step */
  COMM_POPN    = 0x62,  /* Command setup - Partial open */
  COMM_OPN     = 0x63,  /* Command setup - Open */
  COMM_CLS     = 0x64,  /* Command setup - Close */
  COMM_STP     = 0x65,  /* Command setup - STOP */
  COMM_PHOTO   = 0x68,  /* Command setup - Photo */
  COMM_PHOTO2  = 0x69,  /* Command setup - Photo2 */
  COMM_PHOTO3  = 0x6A,  /* Command setup - Photo3 */
  COMM_OPN_STP = 0x6B,  /* Command setup - Stop on opening */
  COMM_CLS_STP = 0x6C,  /* Command setup - Stop on closing */
  IN1          = 0x71,  /* Input setup */
  IN2          = 0x72,  /* Input setup */
  IN3          = 0x73,  /* Input setup */
  IN4          = 0x74,  /* Input setup */
  COMM_LET_OPN = 0x78,  /* Command setup - Obstruction on opening */
  COMM_LET_CLS = 0x79,  /* Command setup - Obstruction on closing */

  AUTOCLS      = 0x80,  /* Main parameters - Auto closing */
  P_TIME       = 0x81,  /* Main parameters - Pause time */
  PH_CLS_ON    = 0x84,  /* Main parameters - Close after Photo - Active */
  PH_CLS_VAR   = 0x86,  /* Main parameters - Close after Photo - Mode */
  PH_CLS_TIME  = 0x85,  /* Main parameters - Close after Photo - Waiting time */
  ALW_CLS_ON   = 0x88,  /* Main parameters - Always close - Active */
  ALW_CLS_VAR  = 0x8A,  /* Main parameters - Always close - Mode */
  ALW_CLS_TIME = 0x89,  /* Main parameters - Always close - Waiting time */
  STAND_BY_ACT = 0x8C,  /* Main parameters - Standby mode - Active ON/OFF */
  WAIT_TIME    = 0x8D,  /* Main parameters - Standby mode - Waiting time */
  STAND_BY_MODE= 0x8E,  /* Main parameters - Standby mode - Mode - safety = 0x00, bluebus = 0x01, all = 0x02 */
  START_ON     = 0x90,  /* Main parameters - Start setup - Active */
  START_TIME   = 0x91,  /* Main parameters - Start setup - Start time */
  SLOW_ON      = 0xA2,  /* Main parameters - Slowdown */
  DIS_VAL      = 0xA4,  /* Position - Value not allowed (disable value) */

  BLINK_ON       = 0x94,  /* Main parameters - Pre-blink - Active */
  BLINK_OPN_TIME = 0x95,  /* Main parameters - Pre-blink - Time on opening */
  BLINK_CLS_TIME = 0x99,  /* Main parameters - Pre-blink - Time on closing */
  OP_BLOCK       = 0x9A,  /* Main parameters - Motor blocking (Operator block) */
  KEY_LOCK       = 0x9C,  /* Main parameters - Button locking */
  T_VAL          = 0xB1,  /* Alarm threshold value — Maintenance threshold (maneuver count) */
  P_COUNT        = 0xB2,  /* Partial count — Dedicated counter */
  C_MAIN         = 0xB4,  /* Cancel maintenance */
  DIAG_BB        = 0xD0,  /* DIAGNOSTICS of BlueBUS devices */
  INF_IO         = 0xD1,  /* State of inputs/outputs */
  DIAG_PAR       = 0xD2,  /* DIAGNOSTICS of other parameters */

  CUR_MAN  = 0x02,  // Current maneuver
  SUBMNU   = 0x04,  // Submenu
  STA      = 0xC0,  // status while moving
  MAIN_SET = 0x80,  // Main parameters
  RUN      = 0x82,  // Command to execute
};

/* run cmd — 11th byte of EVT packets */
enum run_cmd : uint8_t {
  SET          = 0xA9, /* request to change parameters */
  GET          = 0x99, /* request to get parameters */
  GET_SUPP_CMD = 0x89, /* get supported commands */
};

/* Command to be executed.
   11th byte of a CMD packet
   Used in requests and responses
*/
enum control_cmd : uint8_t {
  SBS       = 0x01, /* Step by Step */
  STOP      = 0x02, /* Stop */
  OPEN      = 0x03, /* Open */
  CLOSE     = 0x04, /* Close */
  P_OPN1    = 0x05, /* Partial opening 1 — wicket mode */
  P_OPN2    = 0x06, /* Partial opening 2 */
  P_OPN3    = 0x07, /* Partial opening 3 */
  RSP       = 0x19, /* interface response confirming command receipt */
  EVT       = 0x29, /* interface response sending requested information */

  P_OPN4    = 0x0B, /* Partial opening 4 — Collective */
  P_OPN5    = 0x0C, /* Partial opening 5 — Priority step-by-step */
  P_OPN6    = 0x0D, /* Partial opening 6 — Open and lock */
  UNLK_OPN  = 0x19, /* Unlock and open */
  CLS_LOCK  = 0x0E, /* Close and lock */
  UNLCK_CLS = 0x1A, /* Unlock and close */
  LOCK      = 0x0F, /* Lock */
  UNLOCK    = 0x10, /* Unlock */
  LIGHT_TIMER = 0x11, /* Light timer */
  LIGHT_SW    = 0x12, /* Light on/off */
  HOST_SBS    = 0x13, /* Master SBS */
  HOST_OPN    = 0x14, /* Master open */
  HOST_CLS    = 0x15, /* Master close */
  SLAVE_SBS   = 0x16, /* Slave SBS */
  SLAVE_OPN   = 0x17, /* Slave open */
  SLAVE_CLS   = 0x18, /* Slave close */
  AUTO_ON     = 0x1B, /* Auto-opening active */
  AUTO_OFF    = 0x1C, /* Auto-opening inactive */
};

/*
Information to better understand the packet structure in the protocol
*/
// CMD request packet body
// packets with body size 0x0C = 12 bytes
/*
struct packet_cmd_body_t {
  uint8_t byte_55;     // Header, always 0x55
  uint8_t pct_size1;   // packet body size (without header and CRC; total bytes minus three), for commands = 0x0C
  uint8_t for_series;  // series — recipient, 0xFF = all
  uint8_t for_address; // address — recipient, 0xFF = all
  uint8_t from_series; // series — sender
  uint8_t from_address;// address — sender
  uint8_t mes_type;    // message type, 1 = CMD, 8 = INF
  uint8_t mes_size;    // number of following bytes excluding the two CRC bytes at the end, for commands = 5
  uint8_t crc1;        // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu;     // Command menu; cmd_mnu = 1 for control commands
  uint8_t setup_submnu;// Submenu; together with the command group defines the type of message sent
  uint8_t control_cmd; // Command to be executed
  uint8_t offset;      // Offset for responses; affects requests like the list of supported commands
  uint8_t crc2;        // CRC2, XOR of the previous four bytes
  uint8_t pct_size2;   // packet body size (without header and CRC; total bytes minus three), for commands = 0x0C
};

// RSP response packet body
// packets with body size >= 0x0E = 14 bytes
struct packet_rsp_body_t {
  uint8_t byte_55;     // Header, always 0x55
  uint8_t pct_size1;   // packet body size (without header and CRC; total bytes minus three), >= 0x0E
  uint8_t to_series;   // series — recipient, 0xFF = all
  uint8_t to_address;  // address — recipient, 0xFF = all
  uint8_t from_series; // series — sender
  uint8_t from_address;// address — sender
  uint8_t mes_type;    // message type, for these packets always 8 = INF
  uint8_t mes_size;    // number of following bytes excluding the two CRC bytes at the end
  uint8_t crc1;        // CRC1, XOR of the previous six bytes
  uint8_t cmd_mnu;     // Command menu
  uint8_t sub_inf_cmd; // From which submenu the command was received.
                       // The value is less by 0x80 than the original submenu
  uint8_t sub_run_cmd; // Which command was received.
                       // The value is greater by 0x80 than the received command
  uint8_t hb_data;     // data, high byte
  uint8_t lb_data;     // data, low byte
  uint8_t err;         // Errors
  uint8_t crc2;        // CRC2, XOR of the previous four bytes
  uint8_t pct_size2;   // packet body size (without header and CRC; total bytes minus three), >= 0x0E
};

// EVT response packet body with data
struct packet_evt_body_t {
  uint8_t byte_55;     // Header, always 0x55
  uint8_t pct_size1;   // packet body size (without header and CRC; total bytes minus three), >= 0x0E
  uint8_t to_series;   // series — recipient, 0xFF = all
  uint8_t to_address;  // address — recipient, 0xFF = all
  uint8_t from_series; // series — sender
  uint8_t from_address;// address — sender
  uint8_t mes_type;    // message type, for these packets always 8 = INF
  uint8_t mes_size;    // number of following bytes excluding the two CRC bytes at the end
  uint8_t crc1;        // CRC1, XOR of the previous six bytes
  uint8_t whose;       // Whose packet. Options: 00 - common, 04 - drive controller, 0A - OXI receiver
  uint8_t setup_submnu;// From which submenu the command was received. Equal to the original submenu
  uint8_t sub_run_cmd; // Which command we are replying to. The value is less by 0x80 than the command sent earlier
  uint8_t next_data;   // Next data block
  uint8_t err;         // Errors
  uint8_t data_blk;    // Data block; may occupy several bytes
  uint8_t crc2;        // CRC2, XOR of all previous bytes up to the ninth (Whose packet)
  uint8_t pct_size2;   // packet body size (without header and CRC; total bytes minus three), >= 0x0E
};
*/

enum position_hook_type : uint8_t {
  IGNORE    = 0x00,
  STOP_UP   = 0x01,
  STOP_DOWN = 0x02
};

// Create a class inheriting from Component and Cover
class NiceBusT4 : public Component, public Cover {
 public:
  // drive settings
  bool autocls_flag;      // Auto closing — L1
  bool photocls_flag;     // Close after photo — L2
  bool alwayscls_flag;    // Always close — L3
  bool init_ok = false;   // drive detection at power-up
  bool is_walky = false;  // Walky uses a different position request command
  bool is_robus = false;  // Robus doesn't require periodic position requests
  bool is_ro = false;     // RO600 uses different packets for position status and movement status

  void setup() override;
  void loop() override;
  void dump_config() override; // for logging information about the equipment

  void send_raw_cmd(std::string data);
  void send_cmd(uint8_t data) { this->tx_buffer_.push(gen_control_cmd(data)); }
  void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,
                    std::string next_data, bool data_on, std::string data_command); // long command
  void set_mcu(std::string command, std::string data_command); // command to the motor controller

  void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }

//  void set_update_interval(uint32_t update_interval) {  // interval for getting drive status
//    this->update_interval_ = update_interval;
//  }

  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;
  void send_command_(const uint8_t *data, uint8_t len);
  void request_position(void);         // Request the conditional current position of the drive
  void update_position(uint16_t newpos); // Update the current position of the drive

  uint32_t last_position_time{0};  // Time of the last current position update
  uint32_t update_interval_{500};
  uint32_t last_update_{0};
  uint32_t last_uart_byte_{0};

  CoverOperation last_published_op; // Last published operation/state
  float last_published_pos{-1};

  void publish_state_if_changed(void);

  uint8_t  position_hook_type{IGNORE}; // Flag and position for setting a target position hook
  uint16_t position_hook_value;

  uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//  uint8_t last_init_command_;

  bool init_cu_flag  = false;
  bool init_oxi_flag = false;

  // UART variables
  uint8_t  _uart_nr;
  uart_t*  _uart = nullptr;
  uint16_t _max_opn = 0;      // maximum encoder or timer position
  uint16_t _pos_opn = 2048;   // opening position by encoder or timer; not for all drives
  uint16_t _pos_cls = 0;      // closing position by encoder or timer; not for all drives
  uint16_t _pos_usl = 0;      // conditional current position by encoder or timer; not for all drives
  // header settings for the packet being formed
  uint8_t addr_from[2] = {0x00, 0x66}; // sender address, BusT4 gateway address
  uint8_t addr_to[2];                  // recipient address, the drive controller we control
  uint8_t addr_oxi[2];                 // OXI receiver address

  std::vector<uint8_t> raw_cmd_prepare(std::string data); // prepare user-entered data for sending

  // generate INF commands
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose,
                                   const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data,
                                   const std::vector<uint8_t> &data, size_t len); // all fields
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0);
  } // for commands without data
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd,
                                   const uint8_t next_data, std::vector<uint8_t> data) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());
  } // for commands with data
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose,
                                   const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data) {
    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);
  } // for commands with address and without data

  // generate CMD commands
  std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);

  void init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device);
  void send_array_cmd(std::vector<uint8_t> data);
  void send_array_cmd(const uint8_t *data, size_t len);

  void parse_status_packet(const std::vector<uint8_t> &data); // parse a status packet

  void handle_char_(uint8_t c);                    // handler of a received byte
  void handle_datapoint_(const uint8_t *buffer, size_t len); // handler of received data
  bool validate_message_();                        // function to validate the received message

  std::vector<uint8_t> rx_message_;                // accumulates the received message byte-by-byte
  std::queue<std::vector<uint8_t>> tx_buffer_;     // queue of commands to send
  bool ready_to_tx_{true};                         // flag indicating it's possible to send commands

  std::vector<uint8_t> manufacturer_ = {0x55, 0x55}; // unknown manufacturer at initialization
  std::vector<uint8_t> product_;
  std::vector<uint8_t> hardware_;
  std::vector<uint8_t> firmware_;
  std::vector<uint8_t> description_;
  std::vector<uint8_t> oxi_product;
  std::vector<uint8_t> oxi_hardware;
  std::vector<uint8_t> oxi_firmware;
  std::vector<uint8_t> oxi_description;

}; // class

} // namespace bus_t4
} // namespace esphome
