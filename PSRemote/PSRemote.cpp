#include "WProgram.h"
#include <avr/pgmspace.h>

#include "PSRemote.h"
#include "Max3421e.h"
#include "Usb.h"


#define ARRAY_LENGTH(array) (sizeof(array)/sizeof(*array))


#define PSREMOTE_DEBUG 0
// {{{
#if PSREMOTE_DEBUG
void Serial_print_P(const prog_char *str) {
    char c;

    if (!str) return;
    while ((c = pgm_read_byte(str++)) != 0) {
        Serial.print(c, BYTE);
    }
}

void Serial_println_P(const prog_char *str) {
    Serial_print_P(str);
    Serial.println("");
}

#define DEBUG_PRINT(c, f)   Serial.print(c, f)
#define DEBUG_PRINTLN(c, f) Serial.println(c, f)
#define DEBUG_PRINT_P(c)    Serial_print_P(c)
#define DEBUG_PRINTLN_P(c)  Serial_println_P(c)
#else
#define DEBUG_PRINT(c, f)
#define DEBUG_PRINTLN(c, f)
#define DEBUG_PRINT_P(c)
#define DEBUG_PRINTLN_P(c)
#endif
// }}}


MAX3421E Max;
USB Usb;


/* PS Remote States */
static unsigned int buttonstate;
static unsigned int oldbuttonstate;
static unsigned char oldpsbuttonstate;
	
/* PS Remote Reports */
uint8_t feature_F4_report[] = {0x42, 0x03, 0x00, 0x00};
uint8_t output_01_report[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x02, 0xff, 0x27, 0x10, 0x00, 0x32, 0xff, 
                               0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00, 
                               0x32, 0xff, 0x27, 0x10, 0x00, 0x32, 0x00, 0x00, 
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

PSRemote::PSRemote(void) {
    l2cap_state_ = L2CAP_DOWN_STATE;
    l2cap_txid_ = 1;
    command_scid_ = 0x0040;     // L2CAP local CID for HID_Control
    interrupt_scid_ = 0x0041;   // L2CAP local CID for HID_Interrupt

    hid_flags_ = 0;
    hid_buttons_ = 0;
    old_hid_buttons_ = 0;
    hid_buttons_click_ = 0;
    psremote_status_ = 0;
	
}

PSRemote::~PSRemote(void) {
}

void PSRemote::init(void) {
    Max.powerOn();
    delay(200);
}

void PSRemote::task(void) {
    Max.Task();
    Usb.Task();
	delay(1);

    // re-initialize
    if (Usb.getUsbTaskState() == USB_DETACHED_SUBSTATE_INITIALIZE) {
        /* TODO */
        psremote_status_ = 0;
    }

    // wait for addressing state
    if (Usb.getUsbTaskState() == USB_STATE_CONFIGURING) {
        initBTController();

        if (psremote_status_ & PSREMOTE_STATE_USB_CONFIGURED) {
            hci_state_ = HCI_INIT_STATE;
            hci_counter_ = 10;
            l2cap_state_ = L2CAP_DOWN_STATE;

            Usb.setUsbTaskState(USB_STATE_RUNNING);
        }
    }

    if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
		HCI_task();     // poll the HCI event pipe
		L2CAP_task();   // start polling the ACL input pipe too, though discard data until connected
    }

} // task

/************************************************************/
/* Initialize Bluetooth USB Controller (CSR)                */
/************************************************************/

void PSRemote::initBTController(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};
    USB_DEVICE_DESCRIPTOR *device_descriptor;

    /* initialize data structures for endpoints of device 1 */

    // copy endpoint 0 parameters
    ep_record_[ CONTROL_PIPE ] = *( Usb.getDevTableEntry(0,0) );

    // Bluetooth event endpoint
    ep_record_[ EVENT_PIPE ].epAddr = 0x01;
    ep_record_[ EVENT_PIPE ].Attr = EP_INTERRUPT;
    ep_record_[ EVENT_PIPE ].MaxPktSize = INT_MAXPKTSIZE;
    ep_record_[ EVENT_PIPE ].Interval = EP_POLL;
    ep_record_[ EVENT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ EVENT_PIPE ].rcvToggle = bmRCVTOG0;

    // Bluetoth data endpoint
    ep_record_[ DATAIN_PIPE ].epAddr = 0x02;
    ep_record_[ DATAIN_PIPE ].Attr = EP_BULK;
    ep_record_[ DATAIN_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record_[ DATAIN_PIPE ].Interval = 0;
    ep_record_[ DATAIN_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ DATAIN_PIPE ].rcvToggle = bmRCVTOG0;

    // Bluetooth data endpoint
    ep_record_[ DATAOUT_PIPE ].epAddr = 0x02;
    ep_record_[ DATAOUT_PIPE ].Attr = EP_BULK;
    ep_record_[ DATAOUT_PIPE ].MaxPktSize = BULK_MAXPKTSIZE;
    ep_record_[ DATAOUT_PIPE ].Interval = 0;
    ep_record_[ DATAOUT_PIPE ].sndToggle = bmSNDTOG0;
    ep_record_[ DATAOUT_PIPE ].rcvToggle = bmRCVTOG0;

    // plug kbd.endpoint parameters to devtable
    Usb.setDevTableEntry(BT_ADDR, ep_record_);

    // read the device descriptor and check VID and PID
    rcode = Usb.getDevDescr(BT_ADDR,
                            ep_record_[ CONTROL_PIPE ].epAddr,
                            DEV_DESCR_LEN,
                            (char *) buf);
    if (rcode) {
        DEBUG_PRINT_P( PSTR("\r\nDevice Descriptor Error: ") );
        DEBUG_PRINT(rcode, HEX);
        return;
    }

    device_descriptor = (USB_DEVICE_DESCRIPTOR *) &buf;
    if ((device_descriptor->idVendor != CSR_VID) ||
        (device_descriptor->idProduct != CSR_PID)) {
        DEBUG_PRINT_P( PSTR("\r\nWrong USB Device ID: ") );
        DEBUG_PRINT_P( PSTR("\r\n\t Vendor ID = ") );
        DEBUG_PRINT(device_descriptor->idVendor, HEX);
        DEBUG_PRINT_P( PSTR("\r\n\tProduct ID = ") );
        DEBUG_PRINT(device_descriptor->idProduct, HEX);
        return;
    }

    psremote_status_ |= PSREMOTE_STATE_USB_AUTHORIZED;

    // configure device
    rcode = Usb.setConf(BT_ADDR,
                        ep_record_[ CONTROL_PIPE ].epAddr,
                        BT_CONFIGURATION);
    if (rcode) {
        DEBUG_PRINT_P( PSTR("\r\nDevice Configuration Error: ") );
        DEBUG_PRINT(rcode, HEX);
        return;
    }
    psremote_status_ |= PSREMOTE_STATE_USB_CONFIGURED;

    //LCD.clear();

    DEBUG_PRINT_P( PSTR("\r\nCSR Initialized") );
    //delay(200);
} // initBTController

/************************************************************/
/* HCI Flow Control                                         */
/************************************************************/

void PSRemote::HCI_task(void) {
    HCI_event_task();

    switch (hci_state_) {
      case HCI_INIT_STATE:
        // wait until we have looped 10 times to clear any old events
        if (hci_timeout) {
            hci_reset();
            hci_state_ = HCI_RESET_STATE;
            hci_counter_ = 1000;
        }
        break;

      case HCI_RESET_STATE:
        if (hci_command_complete) {
            DEBUG_PRINT_P( PSTR("\r\nHCI Reset complete") );
            hci_write_scan_enable(SCAN_ENABLE_NOINQ_ENPAG);
			hci_state_ = HCI_CONNECT_IN_STATE;
        }
        if (hci_timeout) {
            DEBUG_PRINT_P( PSTR("\r\nNo response to HCI Reset") );
            hci_state_ = HCI_INIT_STATE;
            hci_counter_ = 10;
        }
        break;
		           
      case HCI_CONNECT_IN_STATE:
        if(hci_incoming_connect_request) {
          hci_accept_connection(psremote_bdaddr_);
          DEBUG_PRINT_P( PSTR("\r\nPS Remote Connected") );
		  hci_state_ = HCI_CHANGE_CONNECTION ; 
        }
        break;
		
	  case HCI_CHANGE_CONNECTION:
		if (hci_connect_complete) {
			hci_change_connection_type();
			
			hci_state_ = HCI_CONNECTED_STATE; 
			l2cap_state_ = L2CAP_INIT_STATE;
			psremote_status_ = PSREMOTE_STATE_CONNECTED;
		}
		

      case HCI_CONNECTED_STATE:
        if (hci_disconn_complete) {
            DEBUG_PRINT_P( PSTR("\r\nPS Remote Disconnected") );
            hci_state_ = HCI_INIT_STATE;
            hci_counter_ = 10;
            l2cap_state_ = L2CAP_DOWN_STATE;
            psremote_status_ &= ~PSREMOTE_STATE_CONNECTED;
        }
        break;

      default:
        break;
    }   // switch (hci_state_)

    return;
} //HCI_task

void PSRemote::HCI_event_task(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    // check input on the event pipe (endpoint 1)
    rcode = Usb.inTransfer(BT_ADDR, ep_record_[ EVENT_PIPE ].epAddr,
                           MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT);
						   
    //DEBUG_PRINT_P( PSTR("\r\nHCI_event_task: rcode = 0x") );
    //DEBUG_PRINT(rcode, HEX);
    if (!rcode) {
        /*  buf[0] = Event Code                            */
        /*  buf[1] = Parameter Total Length                */
        /*  buf[n] = Event Parameters based on each event  */
        DEBUG_PRINT_P( PSTR("\r\nHCI event = 0x") );
        DEBUG_PRINT(buf[0], HEX);
        switch (buf[0]) {   // switch on event type
          case HCI_EVENT_COMMAND_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_COMMAND_COMPLETE;
#if PSREMOTE_DEBUG
			DEBUG_PRINT_P( PSTR("\r\nCommand OK = 0x") );
            DEBUG_PRINT(buf[3], HEX);
			DEBUG_PRINT_P( PSTR(" Returned = 0x") );
            DEBUG_PRINT(buf[5], HEX);
#endif
            break;

          case HCI_EVENT_COMMAND_STATUS:
            hci_event_flag_ |= HCI_FLAG_COMMAND_STATUS;

#if PSREMOTE_DEBUG
            if (buf[2]) {   // show status on serial if not OK
                DEBUG_PRINT_P( PSTR("\r\nHCI Command Failed: ") );
                DEBUG_PRINT_P( PSTR("\r\n\t             Status = ") );
                DEBUG_PRINT(buf[2], HEX);

                DEBUG_PRINT_P( PSTR("\r\n\tCommand_OpCode(OGF) = ") );
                DEBUG_PRINT( ((buf[5] & 0xFC) >> 2), HEX);

                DEBUG_PRINT_P( PSTR("\r\n\tCommand_OpCode(OCF) = ") );
                DEBUG_PRINT( (buf[5] & 0x03), HEX);
                DEBUG_PRINT(buf[4], HEX);
            }
#endif
            break;
			
          case HCI_EVENT_CONNECT_COMPLETE:
            if (!buf[2]) {  // check if connected OK
                // store the handle for the ACL connection
                hci_handle_ = buf[3] | ((buf[4] & 0x0F) << 8);
				
				hci_event_flag_ |= HCI_FLAG_CONNECT_COMPLETE;
			}
			else {
			  DEBUG_PRINT_P( PSTR("\r\nError on Connect Complete = 0x") );
              DEBUG_PRINT(buf[2], HEX);
			}
            break;

          case HCI_EVENT_NUM_COMPLETED_PKT:
#if PSREMOTE_DEBUG
            DEBUG_PRINT_P( PSTR("\r\nHCI Number Of Completed Packets Event: ") );
            DEBUG_PRINT_P( PSTR("\r\n\tNumber_of_Handles = 0x") );
            DEBUG_PRINT(buf[2], HEX);
            for (uint8_t i = 0; i < buf[2]; i++) {
                DEBUG_PRINT_P( PSTR("\r\n\tConnection_Handle = 0x") );
                DEBUG_PRINT((buf[3+i] | ((buf[4+i] & 0x0F) << 8)), HEX);
            }
#endif
            break;

          case HCI_EVENT_QOS_SETUP_COMPLETE:
            break;

          case HCI_EVENT_DISCONN_COMPLETE:
            hci_event_flag_ |= HCI_FLAG_DISCONN_COMPLETE;
            DEBUG_PRINT_P( PSTR("\r\nHCI Disconnection Complete Event: ") );
            DEBUG_PRINT_P( PSTR("\r\n\t           Status = 0x") );
            DEBUG_PRINT(buf[2], HEX);
            DEBUG_PRINT_P( PSTR("\r\n\tConnection_Handle = 0x") );
            DEBUG_PRINT((buf[3] | ((buf[4] & 0x0F) << 8)), HEX);
            DEBUG_PRINT_P( PSTR("\r\n\t           Reason = 0x") );
            DEBUG_PRINT(buf[5], HEX);
            break;
			
		  case HCI_EVENT_CONNECT_REQUEST:
            hci_event_flag_ |= HCI_FLAG_INCOMING_REQUEST;
            DEBUG_PRINT_P( PSTR("\r\nConnection Requested by BD_ADDR: ") );
            for (uint8_t i = 0; i < 6; i++) {
                psremote_bdaddr_[i] = (unsigned char)  buf[2+i];
                DEBUG_PRINT(psremote_bdaddr_[i], HEX);
                if (i < 5)DEBUG_PRINT_P( PSTR(":") );
            }
			
			DEBUG_PRINT_P( PSTR(" LINK: 0x") );
			DEBUG_PRINT(buf[11], HEX);
 
            break;
			
		  case HCI_EVENT_ROLE_CHANGED:
#if PSREMOTE_DEBUG
            DEBUG_PRINT_P( PSTR("\r\nRole Change STATUS: 0x") );
			DEBUG_PRINT(buf[2], HEX);
			
            DEBUG_PRINT_P( PSTR(" BD_ADDR: ") );
            for (uint8_t i = 0; i < 6; i++) {
                DEBUG_PRINT((unsigned char)  buf[3+i], HEX);
                if (i < 5)DEBUG_PRINT_P( PSTR(":") );
            }
			
			DEBUG_PRINT_P( PSTR(" ROLE: 0x") );
			DEBUG_PRINT(buf[9], HEX);
#endif
			break;

          case HCI_EVENT_CHANGED_CONNECTION_TYPE: 
#if PSREMOTE_DEBUG
            DEBUG_PRINT_P( PSTR("\r\nPacket Type Changed STATUS: 0x") );
			DEBUG_PRINT(buf[2], HEX);
            DEBUG_PRINT_P( PSTR(" TYPE: ") );
			DEBUG_PRINT((buf[5] | (buf[6] << 8)), HEX);
#endif
            break;

          case HCI_EVENT_PAGE_SR_CHANGED: 
            break;
			
          default:
            DEBUG_PRINT_P( PSTR("\r\nUnmanaged Event: 0x") );
            DEBUG_PRINT(buf[0], HEX);
            break;
        }   // switch (buf[0])
    }
    return;
} // HCI_event_task

/************************************************************/
/* HCI Commands                                             */
/************************************************************/

uint8_t PSRemote::hci_reset(void) {
    uint8_t buf[3] = {0};

    hci_event_flag_ = 0;    // clear all the flags

    buf[0] = HCI_OCF_RESET;
    buf[1] = HCI_OGF_CTRL_BBAND;
    buf[2] = 0x00; // Parameter Total Length = 0

    return HCI_Command(3, buf);
}

uint8_t PSRemote::hci_write_scan_enable(uint8_t conf) {
   uint8_t buf[4] = {0};

   hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;
   
   buf[0] = HCI_OCF_WRITE_SCAN_ENABLE;
   buf[1] = HCI_OGF_CTRL_BBAND;
   buf[2] = 0x01;
   buf[3] = conf;
   return HCI_Command(4 , buf);
}

uint8_t PSRemote::hci_accept_connection(uint8_t *bdaddr) {
   uint8_t buf[10] = {0};
	
   hci_event_flag_ &= ~(HCI_FLAG_INCOMING_REQUEST);
   
   buf[0] = HCI_OCF_ACCEPT_CONNECTION; // HCI OCF = 9
   buf[1] = HCI_OGF_LINK_CNTRL; // HCI OGF = 1
   buf[2] = 0x07; // parameter length 7
   buf[3] = *bdaddr;  // 6 octet bluetooth address
   buf[4] = *(bdaddr + 1); 
   buf[5] = *(bdaddr + 2);
   buf[6] = *(bdaddr + 3);
   buf[7] = *(bdaddr + 4);
   buf[8] = *(bdaddr + 5);
   buf[9] = 1; //switch role to slave
   
   return HCI_Command(10 , buf);
}

uint8_t PSRemote::hci_change_connection_type(void) {
   uint8_t buf[7] = {0};

   hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;
   
   buf[0] = HCI_OCF_CHANGE_CONNECTION_TYPE;
   buf[1] = HCI_OGF_LINK_CNTRL;
   buf[2] = 0x04; 							// parameter length 4
   buf[3] = (uint8_t) (hci_handle_ & 0xff);	// HCI handle with PB,BC flag
   buf[4] = (uint8_t) (hci_handle_ >> 8);
   buf[5] = 0x18;							// Packet Type: 0xcc18
   buf[6] = 0xcc;
   
   return HCI_Command(7 , buf);
}

/* perform HCI Command */
uint8_t PSRemote::HCI_Command(uint16_t nbytes, uint8_t *dataptr) {
    //hci_event_flag_ &= ~HCI_FLAG_COMMAND_COMPLETE;
    return Usb.ctrlReq(BT_ADDR,
                       ep_record_[ CONTROL_PIPE ].epAddr,
                       bmREQ_HCI_OUT,
                       HCI_COMMAND_REQ,
                       0x00,
                       0x00,
                       0,
                       nbytes,
                       (char *) dataptr);
}

/************************************************************/
/* L2CAP Flow Control                                       */
/************************************************************/

void PSRemote::L2CAP_task(void) {
    L2CAP_event_task();

    switch (l2cap_state_) {
      case L2CAP_DOWN_STATE:
        break;

      case L2CAP_INIT_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_I") );
        l2cap_event_status_ = 0;
		psremote_status_ = 0;
		l2cap_state_ = L2CAP_CONTROL_CONNECTING_STATE;
		
      case L2CAP_CONTROL_CONNECTING_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_C1") );
        if (l2cap_command_connected) {
            l2cap_event_status_ &= ~L2CAP_EV_COMMAND_CONFIGURED;
            l2cap_state_ = L2CAP_CONTROL_REQUEST_STATE;
        }
        break;

      case L2CAP_CONTROL_REQUEST_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_C2") );
        if (l2cap_command_request) {
            l2cap_configure(command_dcid_);
            l2cap_state_ = L2CAP_CONTROL_CONFIGURING_STATE;
        }
        break;

      case L2CAP_CONTROL_CONFIGURING_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_C3") );
        if (l2cap_command_configured) {
            l2cap_state_ = L2CAP_INTERRUPT_CONNECTING_STATE;
        }
        break;


      case L2CAP_INTERRUPT_CONNECTING_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_I1") );
        if (l2cap_interrupt_connected) {
            l2cap_event_status_ &= ~L2CAP_EV_INTERRUPT_CONFIGURED;
            l2cap_state_ = L2CAP_INTERRUPT_REQUEST_STATE;
        }
        break;

      case L2CAP_INTERRUPT_REQUEST_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_I2") );
        if (l2cap_interrupt_request) {
            l2cap_configure(interrupt_dcid_);
            l2cap_state_ = L2CAP_INTERRUPT_CONFIGURING_STATE;
        }
        break;

      case L2CAP_INTERRUPT_CONFIGURING_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_I3") );
        if (l2cap_interrupt_configured) {
            l2cap_state_ = L2CAP_CONNECTED_STATE;
        }
        break;

    /* Established L2CAP */

      case L2CAP_CONNECTED_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_S") );
        hid_flags_ = 0;
		
        initPSController();
		l2cap_state_ = L2CAP_LED_STATE;
        break;
		
	  case L2CAP_LED_STATE:
		if (hid_command_success) {
			LED(psLEDA);
			l2cap_state_ = L2CAP_READY_STATE;
            psremote_status_ = PSREMOTE_STATE_RUNNING;
		}
		break;

      case L2CAP_READY_STATE:
        if (l2cap_interrupt_disconnected || l2cap_command_disconnected) {
            l2cap_state_ = L2CAP_DISCONNECT_STATE;
            psremote_status_ &= ~PSREMOTE_STATE_RUNNING;
        }
        break;

      case L2CAP_DISCONNECT_STATE:
        DEBUG_PRINT_P( PSTR("\r\nL2CAP_D") );
        hci_event_flag_ |= HCI_FLAG_DISCONN_COMPLETE;
        break;

      default:
        break;
    }
    return;
} // L2CAP_task

void PSRemote::L2CAP_event_task(void) {
    uint8_t rcode = 0;  // return code
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    // check input on the event pipe (endpoint 2)
    rcode = Usb.inTransfer(BT_ADDR, ep_record_[ DATAIN_PIPE ].epAddr,
                           MAX_BUFFER_SIZE, (char *) buf, USB_NAK_NOWAIT);
						   
    if (!rcode) {
        if (acl_handle_ok) {
            if      (l2cap_control) {
                DEBUG_PRINT_P( PSTR("\r\nL2CAP Signaling Command = 0x") );
                DEBUG_PRINT(buf[8], HEX);
				if      (l2cap_command_reject) {
					DEBUG_PRINT_P( PSTR("\r\nID = 0x") );
					DEBUG_PRINT(buf[9], HEX);
					DEBUG_PRINT_P( PSTR(" Reason = 0x") );
					DEBUG_PRINT(buf[12] | (buf[13] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" DATA = 0x") );
					DEBUG_PRINT(buf[14] | (buf[15] << 8), HEX);
				}
                else if (l2cap_connection_request) {
					DEBUG_PRINT_P( PSTR("\r\nID = 0x") );
					DEBUG_PRINT(buf[9], HEX);
					DEBUG_PRINT_P( PSTR(" PSM = 0x") );
					DEBUG_PRINT(buf[12] | (buf[13] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" SCID = 0x") );
					DEBUG_PRINT(buf[14] | (buf[15] << 8), HEX);
					
					if ((buf[12] | (buf[13] << 8)) == L2CAP_PSM_WRITE) {
						command_dcid_ = buf[14] | (buf[15] << 8);
						l2cap_connect_response(buf[9], command_scid_, command_dcid_);
						l2cap_event_status_ |= L2CAP_EV_COMMAND_CONNECTED;
					}
					else if ((buf[12] | (buf[13] << 8)) == L2CAP_PSM_READ) {
						interrupt_dcid_ = buf[14] | (buf[15] << 8);
						l2cap_connect_response(buf[9], interrupt_scid_, interrupt_dcid_);
						l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONNECTED;
					}
				}
				else if (l2cap_configuration_request) {
											
					DEBUG_PRINT_P( PSTR("\r\nConf Request ID = 0x") );
					DEBUG_PRINT(buf[9], HEX);
					DEBUG_PRINT_P( PSTR(" LEN = 0x") );
					DEBUG_PRINT(buf[10] | (buf[11] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" SCID = 0x") );
					DEBUG_PRINT(buf[12] | (buf[13] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" FLAG = 0x") );
					DEBUG_PRINT(buf[14] | (buf[15] << 8), HEX);
					
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) { 
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_CONFIG_REQ;
                        l2cap_config_response(buf[9], command_dcid_);
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONFIG_REQ;
                        l2cap_config_response(buf[9], interrupt_dcid_);
                    }
                }
				else if (l2cap_configuration_response) {
					
					DEBUG_PRINT_P( PSTR("\r\nConf Response ID = 0x") );
					DEBUG_PRINT(buf[9], HEX);
					DEBUG_PRINT_P( PSTR(" LEN = 0x") );
					DEBUG_PRINT(buf[10] | (buf[11] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" SCID = 0x") );
					DEBUG_PRINT(buf[12] | (buf[13] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" FLAG = 0x") );
					DEBUG_PRINT(buf[14] | (buf[15] << 8), HEX);
					DEBUG_PRINT_P( PSTR(" RESULT = 0x") );
					DEBUG_PRINT(buf[16] | (buf[17] << 8), HEX);
					
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_CONFIGURED;
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_CONFIGURED;
                    }
                }
                else if (l2cap_disconnect_request) {
					DEBUG_PRINT_P( PSTR("\r\nDisconnect Req  SCID = 0x") );
					DEBUG_PRINT(buf[12] | (buf[13] << 8), HEX);
                    if ((buf[12] | (buf[13] << 8)) == command_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_COMMAND_DISCONNECT_REQ;
                        l2cap_disconnect_response(buf[9], command_scid_, command_dcid_);
                    }
                    else if ((buf[12] | (buf[13] << 8)) == interrupt_scid_) {
                        l2cap_event_status_ |= L2CAP_EV_INTERRUPT_DISCONNECT_REQ;
                        l2cap_disconnect_response(buf[9], command_scid_, command_dcid_);
                    }
                }
			}
            else if (l2cap_interrupt) {
                readReport(buf);
			}
            else if (l2cap_command) {
                if (hid_handshake_success) {
                    hid_flags_ |= HID_FLAG_COMMAND_SUCCESS;
                }
            }
        } // acl_handle_ok
    } // !rcode

    return;
} // L2CAP_event_task

/************************************************************/
/* L2CAP Commands                                           */
/************************************************************/
uint8_t PSRemote::l2cap_connect_response(uint8_t rxid, uint16_t dcid, uint16_t scid) {
    uint8_t cmd_buf[12];
    cmd_buf[0] = L2CAP_CMD_CONNECTION_RESPONSE; // Code
    cmd_buf[1] = rxid;                          // Identifier
    cmd_buf[2] = 0x08;                          // Length
    cmd_buf[3] = 0x00;
    cmd_buf[4] = (uint8_t) (dcid & 0xff);       // Destination CID (Our)
    cmd_buf[5] = (uint8_t) (dcid >> 8);
    cmd_buf[6] = (uint8_t) (scid & 0xff);       // Source CID (PS Remote)
    cmd_buf[7] = (uint8_t) (scid >> 8);
    cmd_buf[8] = 0x00;     					    // Result
    cmd_buf[9] = 0x00;
    cmd_buf[10] = 0x00;     				 	// Status
    cmd_buf[11] = 0x00;

    return L2CAP_Command((uint8_t *) cmd_buf, 12);
}

uint8_t PSRemote::l2cap_configure(uint16_t dcid) {
    uint8_t cmd_buf[12];
    cmd_buf[0] = L2CAP_CMD_CONFIG_REQUEST;  // Code
    cmd_buf[1] = (uint8_t) (l2cap_txid_++); // Identifier
    cmd_buf[2] = 0x08;                      // Length
    cmd_buf[3] = 0x00;
    cmd_buf[4] = (uint8_t) (dcid & 0xff);   // Destination CID
    cmd_buf[5] = (uint8_t) (dcid >> 8);
    cmd_buf[6] = 0x00;                      // Flags
    cmd_buf[7] = 0x00;
    cmd_buf[8] = 0x01;  					// Config Opt: type = MTU (Maximum Transmission Unit)
    cmd_buf[9] = 0x02;  					// Config Opt: length
    cmd_buf[10] = 0x40; 					// Config Opt: data = maximum SDU size is 672 octets
    cmd_buf[11] = 0x00;

    return L2CAP_Command((uint8_t *) cmd_buf, 12);
}

uint8_t PSRemote::l2cap_config_response(uint8_t rxid, uint16_t dcid) {
    uint8_t resp_buf[10];
    resp_buf[0] = L2CAP_CMD_CONFIG_RESPONSE;    // Code
    resp_buf[1] = rxid;                         // Identifier
    resp_buf[2] = 0x06;                         // Length
    resp_buf[3] = 0x00;
    resp_buf[4] = (uint8_t) (dcid & 0xff);      // Source CID
    resp_buf[5] = (uint8_t) (dcid >> 8);
    resp_buf[6] = 0x00;                         // Result
    resp_buf[7] = 0x00;
    resp_buf[8] = 0x00;                         // Config
    resp_buf[9] = 0x00;

    return L2CAP_Command((uint8_t *) resp_buf, 10);
}

uint8_t PSRemote::l2cap_disconnect_response(uint8_t rxid, uint16_t scid, uint16_t dcid) {
    uint8_t resp_buf[8];
    resp_buf[0] = L2CAP_CMD_DISCONNECT_RESPONSE;    // Code
    resp_buf[1] = rxid;                             // Identifier
    resp_buf[2] = 0x04;                             // Length
    resp_buf[3] = 0x00;
    resp_buf[4] = (uint8_t) (dcid & 0xff);          // Destination CID
    resp_buf[5] = (uint8_t) (dcid >> 8);
    resp_buf[6] = (uint8_t) (scid & 0xff);          // Source CID
    resp_buf[7] = (uint8_t) (scid >> 8);

    return L2CAP_Command((uint8_t *) resp_buf, 8);
}

uint8_t PSRemote::L2CAP_Command(uint8_t *data, uint8_t length) {
    uint8_t buf[MAX_BUFFER_SIZE] = {0};
    buf[0] = (uint8_t) (hci_handle_ & 0xff);    // HCI handle with PB,BC flag
    buf[1] = (uint8_t) (((hci_handle_ >> 8) & 0x0f) | 0x20);
    buf[2] = (uint8_t) ((4 + length) & 0xff);   // HCI ACL total data length
    buf[3] = (uint8_t) ((4 + length) >> 8);
    buf[4] = (uint8_t) (length & 0xff);         // L2CAP header: Length
    buf[5] = (uint8_t) (length >> 8);
    buf[6] = 0x01;  // L2CAP header: Channel ID
    buf[7] = 0x00;  // L2CAP Signalling channel over ACL-U logical link
    for (uint8_t i = 0; i < length; i++) {      // L2CAP C-frame
        buf[8+i] = *data;
        data++;
    }

    // output on endpoint 2
    return Usb.outTransfer(BT_ADDR,
                           ep_record_[ DATAOUT_PIPE ].epAddr,
                           (8 + length),
                           (char *) buf);
}

/************************************************************/
/* HID Commands                                             */
/************************************************************/

uint8_t PSRemote::initPSController(void) {
	uint8_t header = 2;
    uint8_t init_buf[header+PS3_F4_REPORT_LEN];
    init_buf[0] = HID_THDR_SET_REPORT_FEATURE;      // THdr
	init_buf[1] = PS3_F4_REPORT_ID;                 // Report ID
	
	uint8_t i;
	for (uint8_t i = 0; i < PS3_F4_REPORT_LEN; i++) {
		init_buf[header+i] = (uint8_t) feature_F4_report[i];
	}
	
    return writeReport((uint8_t *) init_buf, header+PS3_F4_REPORT_LEN);
}

void PSRemote::readReport(uint8_t *data) {
    if (hid_input_report) {
		uint8_t start_pos = 11;
		uint8_t i;
		for (uint8_t i = 0; i < PS3_01_REPORT_LEN; i++) {
			report[i] = data[start_pos+i];
		}
		
		buttonstate = report[ButtonStateL] | (report[ButtonStateH] << 8);
				
		motion[0] = report[AccelXL] | (report[AccelXH] << 8);
		motion[1] = report[AccelYL] | (report[AccelYH] << 8);
		motion[2] = report[AccelZL] | (report[AccelZH] << 8);
		motion[3] = report[GyroZL]  | (report[GyroZH] << 8);
		
		statusReportReceived = true;
    }
	   
	else {
		DEBUG_PRINT_P( PSTR("\r\nUnmanaged Input Report: THDR 0x") );
		DEBUG_PRINT(data[8], HEX);
		DEBUG_PRINT_P( PSTR(" ID 0x") );
		DEBUG_PRINT(data[9], HEX);
	}
} // readReport

uint8_t PSRemote::writeReport(uint8_t *data, uint8_t length) {
    uint8_t buf[MAX_BUFFER_SIZE] = {0};

    buf[0] = (uint8_t) (hci_handle_ & 0xff);    // HCI handle with PB,BC flag
    buf[1] = (uint8_t) (((hci_handle_ >> 8) & 0x0f) | 0x20);
    buf[2] = (uint8_t) ((4 + length) & 0xff);   // HCI ACL total data length
    buf[3] = (uint8_t) ((4 + length) >> 8);
    buf[4] = (uint8_t) (length & 0xff);   // L2CAP header: Length
    buf[5] = (uint8_t) (length >> 8);
    buf[6] = (uint8_t) (command_dcid_ & 0xff);  // L2CAP header: Channel ID
    buf[7] = (uint8_t) (command_dcid_ >> 8);
    for (uint8_t i = 0; i < length; i++) {
        buf[8+i] = *data;
        data++;
    }

    hid_flags_ &= ~HID_FLAG_COMMAND_SUCCESS;
    // output on endpoint 2
    return Usb.outTransfer(BT_ADDR,
                           ep_record_[ DATAOUT_PIPE ].epAddr,
                           (8 + length),
                           (char *) buf);
} // writeReport

uint8_t PSRemote::LEDRumble(uint8_t ledrum) {
	uint8_t header = 2;
    uint8_t led_buf[header+PS3_01_REPORT_LEN];
	
    led_buf[0] = HID_THDR_SET_REPORT_OUTPUT;      	// THdr
    led_buf[1] = PS3_01_REPORT_ID;                	// Report ID
	
	uint8_t i;
	for (uint8_t i = 0; i < PS3_01_REPORT_LEN; i++) {
		led_buf[header+i] = (uint8_t) output_01_report[i];
	}
	
    led_buf[header+9] = (ledrum & 0x0f) << 1;		//LED Conf
	
    if (ledrum & 0x30) {							//Rumble Conf
		led_buf[header+1] = led_buf[header+3] = 0xfe;
		if (ledrum & 0x10) led_buf[header+4] = 0xff;
		else led_buf[header+2] = 0xff;
    }
	
	oldled = (ledrum & 0x0f);
    oldrumble = (ledrum & 0x30);
	
    return writeReport((uint8_t *) led_buf, header+PS3_01_REPORT_LEN);
}

/************************************************************/
/* PSRemote Commands                                        */
/************************************************************/
unsigned char PSRemote::getStatus(void) {
    return psremote_status_;
}

bool PSRemote::statConnected(void) {
	return (psremote_status_ & PSREMOTE_STATE_RUNNING);
}

bool PSRemote::statReportReceived(void) {
	if (statusReportReceived) {
		statusReportReceived = false;  // clear the report received flag
		return true;
	}
	else return false;
}

unsigned char PSRemote::getJoystick(unsigned char joy) {
	if (joy > 3) return(128); // not valid joystick
    return( *(&report[LeftStickX] + joy));
}

unsigned int PSRemote::getMotion(unsigned char axis) {
	if(axis > 3) return(512); // not valid sensor
    return(motion[axis]);
}

unsigned char PSRemote::getPressure(unsigned char button) {
	if ((button < 4) || (button > 15)) return(0); // not valid pressure
    return(*(&report[PressureUp] + button - 4));
}

bool PSRemote::buttonChanged(void) {
	if ((buttonstate != oldbuttonstate) || (report[PSButtonState] != oldpsbuttonstate)) {
	  oldbuttonstate = buttonstate;
	  oldpsbuttonstate = report[PSButtonState];
	  return(true);
	}
	return(false);
}

bool PSRemote::buttonPressed(unsigned char button) {
	if(button > 16) return(0); // not valid button
	if(button == 16) return( report[PSButtonState] & 0x01);
	return(buttonstate & (1 << button));
	
}

uint8_t PSRemote::LED(uint8_t led) {
	led = (led & 0x0f);
	oldled = led;
	return LEDRumble(led+oldrumble);
}

uint8_t PSRemote::Rumble(uint8_t rumble) {
	rumble = (rumble & 0x30);
    oldrumble = rumble;
	return LEDRumble(rumble+oldled);
}

void PSRemote::getPSADDR(unsigned char * bdaddr){
    for( int i=0; i < 6; i++){
        bdaddr[i] = psremote_bdaddr_[i];
    }
	return;
}

void PSRemote::getBDADDR(unsigned char * bdaddr){
	return;
}

void PSRemote::setBDADDR(unsigned char * bdaddr){
	return;
}

// vim: sts=4 sw=4 ts=4 et cin fdm=marker cms=//%s syntax=arduino