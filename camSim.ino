#include <mavlink_types.h>
#include <mavlink.h>
#include <mavlink_helpers.h>
#include <protocol.h>
#include <checksum.h>

#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;

#define TRIGGER_PWM 1500
#define PWM_IN_PIN 5
int pwm = 0.0f;
int pwm_high = 0.0f;
int pwm_low = 0.0f;
float shutter_duration_start_time = 0.0f;
float shutter_duration = 0.0f;
int pwm_shutter_count = 0;

// Mavlink variables
#define MAVLINK_HEATBEAT_INTERVAL_MILLISEC 1000

unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened

SoftwareSerial softSerial = SoftwareSerial(rxPin, txPin);

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;


  while(softSerial.available()>0) {
    uint8_t c = softSerial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            Serial.println("HB");
          }
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            //mavlink_gps_raw_int_t sys_status;
            //mavlink_msg_gps_raw_int_decode(&msg, &sys_status);
            Serial.println("GPS_RAW");
          }
          break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            //mavlink_gps_raw_int_t sys_status;
            //mavlink_msg_gps_raw_int_decode(&msg, &sys_status);
            Serial.println("GLOBAL_POS");
          }
          break;
          
        case MAVLINK_MSG_ID_ATTITUDE:
          {
            //mavlink_gps_raw_int_t sys_status;
            //mavlink_msg_gps_raw_int_decode(&msg, &sys_status);
            Serial.println("ATTITUDE");
          }
          break;

        case MAVLINK_MSG_ID_SYSTEM_TIME:
          {
            //mavlink_gps_raw_int_t sys_status;
            //mavlink_msg_gps_raw_int_decode(&msg, &sys_status);
            Serial.println("SYSTEM_TIME");
          }
          break;
          
        case MAVLINK_MSG_ID_COMMAND_LONG:
          {
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(&msg, &packet);
            switch(packet.command) {

            case MAV_CMD_DO_DIGICAM_CONTROL:
              //Serial.println("Received Digicam Control");
              break;

            default:
              break;
              
            }
            
          }
          break;
       default:
          break;
      }
    }
  }
}

void send_heartbeat()
{
  /* Simulate as Micasense camera */ 
  int sysid = 2;
  int compid = 100;
  int type = 0;
  uint8_t autopilot_type = 1;
  uint8_t system_mode = MAV_MODE_FLAG_TEST_ENABLED;
  uint32_t custom_mode = 3;
  uint8_t system_state = MAV_STATE_ACTIVE;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  softSerial.write(buf, len);
}

void read_camera_pwm_trigger()
{
  
  // read camera shutter pwm
  pwm = pulseIn(PWM_IN_PIN, HIGH);

  // out of range detection
  if (pwm < 900 || pwm > 2200)
  {
    return;
  }
  
  if(pwm >= TRIGGER_PWM)
  {
    // start timer
    if(shutter_duration_start_time < 1.0f)
    {
      shutter_duration_start_time = micros();
      pwm_high = max(pwm,pwm_high);
    }
    // count duration
    shutter_duration = micros() - shutter_duration_start_time;
  }
  else if (pwm < TRIGGER_PWM
      && (shutter_duration >= 19000.0f)
      && (shutter_duration_start_time > 1.0f))
  {
    
    pwm_low = max(pwm,pwm_low);
    shutter_duration_start_time = 0.0f;
    pwm_shutter_count++;

    Serial.print(pwm_high);
    Serial.print(" ");
    Serial.print(pwm_low);
    Serial.print(" ");
    Serial.print(shutter_duration / 1000.0f);
    Serial.print(" ");
    Serial.println(pwm_shutter_count);
    
  }
}

void setup()
{
  Serial.begin(57600);
  pinMode(PWM_IN_PIN, INPUT);

  // define pin modes for tx, rx:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  // set the data rate for the SoftwareSerial port
  softSerial.begin(57600);
}

void loop()
{

  // Send Heartbeat
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= MAVLINK_HEATBEAT_INTERVAL_MILLISEC) {
    previousMillisMAVLink = currentMillisMAVLink;
    send_heartbeat();
  }

  // Receive mavlink message
  comm_receive();

  // Read camera pwm trigger
  read_camera_pwm_trigger();

}
