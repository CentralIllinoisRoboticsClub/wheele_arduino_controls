// Henry's Bench
// 1st CAN Network - CAN TRANSMIT

#include <mcp_can.h>
#include <SPI.h>


const int SPI_CS_PIN = 10;

//Rx
long unsigned int rxId;
unsigned long rcvTime;
unsigned char len = 0;
unsigned char buf[8];

// Build an ID or PGN

long unsigned int txID = 0x1881ABBA; // This format is typical of a 29 bit identifier.. the most significant digit is never greater than one.
unsigned char stmp[8] = {0x0E, 0x00, 0xFF, 0x22, 0xE9, 0xFA, 0xDD, 0x51};

//Construct a MCP_CAN Object and set Chip Select to 10.

MCP_CAN CAN(SPI_CS_PIN);                            


void setup()
{
    Serial.begin(9600);

    while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 250K
    {
        Serial.println("CAN BUS Module Failed to Initialized");
        Serial.println("Retrying....");
        delay(500);
        
    }
    Serial.println("CAN BUS Shield init ok!");
}


void loop()
{   //Serial.println("In loop");

    // send the data:  id = 0x00, Extended Frame, data len = 8, stmp: data buf
    // Extended Frame = 1.
    
    CAN.sendMsgBuf(txID,1, 8, stmp);    
    delay(1000);    // send data every 25mS

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        rcvTime = millis();
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        rxId= CAN.getCanId();

        Serial.print(rcvTime);
        Serial.print("\t\t");
        Serial.print("0x");
        Serial.print(rxId, HEX);
        Serial.print("\t");

        for(int i = 0; i<len; i++)    // print the data
        {
            if(buf[i] > 15){
              Serial.print("0x");
              Serial.print(buf[i], HEX);    
            }
          else{
              Serial.print("0x0");
              Serial.print(buf[i], HEX);
          }  
            
            //Serial.print("0x");
            //Serial.print(buf[i], HEX);
            
            Serial.print("\t");            
        }
        Serial.println();
    }
}
