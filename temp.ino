extern "C" {
	#include "user_interface.h"  // Required for wifi_station_connect() to work
}

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <stdarg.h>
#include <math.h>
#include "lwip/inet_chksum.h"

typedef struct {
    uint16_t Checksum;
    char ErrorText[ 64 ];
    int ErrorTextLength;
    bool Success;
} __attribute__( ( aligned( 4 ) ) ) LastResultInfo;

void ErrorBlinks( void );
void SuccessBlinks( void );
bool ConnectToWiFi( int Seconds );
bool ConnectToMQTT( void );
bool SendTemperature( void );
void EnterDeepSleep( int SleepTime );
void ReadLastResult( void );
void SetLastError( bool Error, const char* ErrorText, ... );
void SendErrorText( void );
const char* PubSubClient_Error2Str( int Error );
const char* WiFi_Status2Str( int Status );

void WiFiOn( void );
void WiFiOff( void );

const char* Wifi_SSID = "***";
const char* Wifi_PSK = "***";
const char* Hostname = "tara8266-1";

const char* MQTTBrokerIP = "raspberrypi.local";
const char* MQTTBrokerUsername = "tara";
const char* MQTTBrokerPassword = "***";
const int MQTTBrokerPort = 1883;

const char* MQTTTemperatureTopic = "tara/esp8266/tara8266-1/temperature";
const char* MQTTErrorTopic = "tara/esp8266/tara8266-1/error";

const int ErrorBlinkRate = 50;
const int ErrorBlinkTime = 1000;

const int WifiTimeoutSeconds = 15;
const int SleepTimeSeconds = 60;

Adafruit_BMP280 MainSensor;
WiFiClient WifiConnection;

PubSubClient BrokerClient( WifiConnection );

LastResultInfo LastResult;
LastResultInfo ThisResult;

/*
 * Takes a PubSubClient error code and returns the description as a string.
 */
const char* PubSubClient_Error2Str( int Error ) {
    switch ( Error ) {
        case MQTT_CONNECTION_TIMEOUT: return "MQTT: The server did not respond within the keepalive time";
        case MQTT_CONNECTION_LOST: return "MQTT: The connection was lost";
        case MQTT_CONNECT_FAILED: return "MQTT: Failed to connect to server";
        case MQTT_DISCONNECTED: return "MQTT: The client has disconnected";
        case MQTT_CONNECTED: return "MQTT: The client is connected";
        case MQTT_CONNECT_BAD_PROTOCOL: return "MQTT: The server doesn't support the requested version of MQTT";
        case MQTT_CONNECT_BAD_CLIENT_ID: return "MQTT: The server has rejected the client identifier";
        case MQTT_CONNECT_UNAVAILABLE: return "MQTT: The server was unable to accept the connection";
        case MQTT_CONNECT_BAD_CREDENTIALS: return "MQTT: The server rejected your username and or password";
        case MQTT_CONNECT_UNAUTHORIZED: return "MQTT: This client is not authorized to connect";
        default: break;
    };

    return "(Unknown)";
}

/*
 * Takes a WiFi status code and returns it's description as a string.
 */
const char* WiFi_Status2Str( int Status ) {
    switch ( Status ) {
        case WL_IDLE_STATUS: return "Idle";
        case WL_NO_SSID_AVAIL: return "AP SSID Not found";
        case WL_SCAN_COMPLETED: return "Scan completed";
        case WL_CONNECTED: return "Connected";
        case WL_CONNECT_FAILED: return "Connection failed";
        case WL_CONNECTION_LOST: return "Connection lost";
        case WL_DISCONNECTED: return "Disconnected";
        case WL_NO_SHIELD: return "No shield detected";
        default: break;
    };

    return "(Unknown)";
}

/*
 * Reads the RTC memory containing information about our last run.
 */
void ReadLastResult( void ) {
    uint16_t OurSum = 0;
    uint16_t RTCSum = 0;

    if ( ESP.rtcUserMemoryRead( 0, ( uint32_t* ) &LastResult, sizeof( LastResultInfo ) ) == true ) {
        RTCSum = inet_chksum( ( void* ) ( ( ( uint8_t* ) &LastResult ) + 4 ), sizeof( LastResultInfo ) - 4 );
        OurSum = LastResult.Checksum;

        /* If the checksums do not match then that means either RTC memory
         * was corrupted, or this is our first run.
         * In this case just say the last run was a success.
         */
        if ( RTCSum != OurSum ) {
            memset( &LastResult, 0, sizeof( LastResultInfo ) );
            LastResult.Success = true;
        }
    }
}

/*
 * This sets the given error information in RTC memory along
 * with it's checksum so we can check it's validitity on the next boot.
 */
void SetErrorInfo( bool Error, const char* ErrorText, ... ) {
    va_list Argp;

    memset( &ThisResult, 0, sizeof( LastResultInfo ) );

    if ( Error == true ) {
        va_start( Argp, ErrorText );
            ThisResult.ErrorTextLength = vsnprintf( ThisResult.ErrorText, sizeof( ThisResult.ErrorText ), ErrorText, Argp );
        va_end( Argp );

        ThisResult.Success = false;
    } else {
        ThisResult.Success = true;
    }

    ThisResult.Checksum = inet_chksum( ( void* ) ( ( ( uint8_t* ) &ThisResult ) + 4 ), sizeof( LastResultInfo ) - 4 );
    ESP.rtcUserMemoryWrite( 0, ( uint32_t* ) &ThisResult, sizeof( LastResultInfo ) );
}

/*
 * Blinks rapidly for (ErrorBlinkTime) milliseconds at a rate of
 * (ErrorBlinkRate) milliseconds between states.
 */
void ErrorBlinks( void ) {
    unsigned long Timeout = 0;
    bool State = true;

    Timeout = millis( ) + ErrorBlinkTime;

    while ( millis( ) < Timeout ) {
        digitalWrite( LED_BUILTIN, State );
        State = ! State;

        delay( ErrorBlinkRate );
    }

    digitalWrite( LED_BUILTIN, true );
}

/*
 * Simply blinks twice with a second delay between blinks.
 */
void SuccessBlinks( void ) {
    int Count = 2;

    while ( Count-- ) {
        digitalWrite( LED_BUILTIN, false );
        delay( 250 );
        digitalWrite( LED_BUILTIN, true );

        delay( 1000 );
    }
}

/*
 * Turns the WiFi hardware on.
 * From: https://github.com/esp8266/Arduino/issues/3072
 */
void WiFiOn( void ) {
	wifi_fpm_do_wakeup( );
	wifi_fpm_close( );
	wifi_set_opmode( STATION_MODE );
}

/*
 * Turns the WiFi hardware off.
 * From: https://github.com/esp8266/Arduino/issues/3072
 */
void WiFiOff( void ) {
    const int FPM_SLEEP_MAX_TIME = 0xFFFFFFF;

	wifi_set_opmode( NULL_MODE );
	wifi_set_sleep_type( MODEM_SLEEP_T );
	wifi_fpm_open( );
	wifi_fpm_do_sleep( FPM_SLEEP_MAX_TIME );
}

/*
 * Attempts to connect to the configured WiFi access point.
 * Returns true if connected, false on error or if timeout reached.
 */
bool ConnectToWiFi( int TimeoutSeconds ) {
    unsigned long Timeout = 0;
    char Message[ 128 ];
    int Status = 0;

    WiFi.persistent( false );
    WiFi.hostname( Hostname );

    WiFiOff( );
    WiFiOn( );

    if ( WiFi.status( ) != WL_CONNECTED ) {
        WiFi.begin( Wifi_SSID, Wifi_PSK );
    }

    Timeout = millis( ) + ( TimeoutSeconds * 1000 );

    Serial.printf( "Connecting to [%s]...", Wifi_SSID );

    while ( millis( ) < TimeoutSeconds && ( ( Status = WiFi.status( ) != WL_CONNECTED ) ) ) {
        Serial.print( "." );
        delay( 250 );
    }

    if ( Status == WL_CONNECTED ) {
        Serial.printf( " Connected! IP: [%s]\r\n", WiFi.localIP( ).toString( ).c_str( ) );
        return true;
    } 

    snprintf( Message, sizeof( Message ), "Failed to connect to [%s]: %s", Wifi_SSID, WiFi_Status2Str( Status ) );
    SetErrorInfo( true, Message );

    Serial.printf( "%s\r\n", Message );
    return false;
}

/*
 * Attempts to connect to the MQTT broker.
 * If successful it will return true, otherwise it will set
 * the error info in RTC memory and return false.
 */
bool ConnectToMQTT( void ) {
    BrokerClient.setServer( MQTTBrokerIP, MQTTBrokerPort );
    BrokerClient.connect( Hostname, MQTTBrokerUsername, MQTTBrokerPassword );

    if ( BrokerClient.state( ) != MQTT_CONNECTED ) {
        SetErrorInfo( true, PubSubClient_Error2Str( BrokerClient.state( ) ) );
        return false;
    }

    return true;
}

/*
 * Reads the BMP280's temperature value and sends it's value to the MQTT broker.
 * Returns true if the sensor was read successfully.
 */
bool SendTemperature( void ) {
    String TemperatureString;
    float Temperature = 0.0f;

    if ( MainSensor.begin( ) == true ) {
        Temperature = MainSensor.readTemperature( );
        TemperatureString = String( Temperature );

        BrokerClient.publish( MQTTTemperatureTopic, TemperatureString.c_str( ) );

        return true;
    } 

    return false;
}

/* 
 * Puts the ESP8266 into deep sleep for (SleepTimeSeconds).
 */
void EnterDeepSleep( int SleepTime ) {
    Serial.printf( "Sleeping for %d seconds.\r\n", SleepTimeSeconds );
    WiFiOff( );
    ESP.deepSleep( SleepTime * 1000000, WAKE_RF_DEFAULT );
}

void setup( void ) {
    bool LastSendFailed = false;
    bool Success = false;

    Serial.begin( 115200 );
    Serial.println( "\x1B;2j\rReady..." );

    ReadLastResult( );

    if ( LastResult.Success == true ) {
        Serial.println( "*** Last send was successful ***" );
    } else {
        Serial.printf( "*** Last send failed. Reason: %s ***\r\n", LastResult.ErrorText );
        LastSendFailed = true;
    }

    SetErrorInfo( false, "No error" );

    pinMode( LED_BUILTIN, OUTPUT );
    digitalWrite( LED_BUILTIN, true );

    if ( ConnectToWiFi( WifiTimeoutSeconds ) == true ) {
        if ( ConnectToMQTT( ) == true ) {
            SendTemperature( );
            Success = true;

            if ( LastSendFailed == true ) {
                BrokerClient.publish( MQTTErrorTopic, LastResult.ErrorText );
            }

            BrokerClient.disconnect( );
        }

        WifiConnection.stop( );
    }

    /* Blink for some visual feedback */
    if ( Success == true ) {
        SuccessBlinks( );
    } else {
        ErrorBlinks( );
    }

    EnterDeepSleep( SleepTimeSeconds );
}

void loop( void ) {
    Serial.println( "Should not get here" );

    while ( true ) {
        delay( 1000 );
    }
}
