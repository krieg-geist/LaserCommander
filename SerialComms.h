#ifndef SERIAL_COMMS
#define SERIAL_COMMS

#include <Arduino.h>

#define BUFFER_SIZE 1024

/**
 * @enum Command
 * @brief Defines possible commands for controlling laser operations and other serial communications.
 */
enum class Command
{
    // Control commands
    FrameStart = 0x01,
    StopProcessing = 0x03,

    // Laser commands
    TurnOn = 0x10,
    TurnOff = 0x11,
    SendTo = 0x12,
    DrawLine = 0x13,
    DrawRect = 0x14,
    SetColor = 0x15,
    Wait = 0x16
};

/**
 * @brief Retrieves the expected payload size for each command.
 * @param command The command for which the payload size is required.
 * @return byte Size of the expected payload in bytes.
 */
byte getPayloadSize(Command command)
{
    switch (command)
    {
    case Command::FrameStart:
        return 2; // uint16_t frame size
    case Command::TurnOn:
    case Command::TurnOff:
        return 0; // No payload needed
    case Command::SendTo:
        return 4; // Two uint16_t coordinates (x, y)
    case Command::DrawLine:
    case Command::DrawRect:
        return 8; // Four uint16_t coordinates (x1, y1, x2, y2)
    case Command::SetColor:
        return 3; // Three uint8_t values (R, G, B)
    case Command::Wait:
        return 2; // One uint16_t for duration
    default:
        return 0; // Unknown command, no payload assumed
    }
}

class SerialComms
{
public:
    /**
     * @brief Construct a new SerialComms object.
     * @param laserRef Reference to the associated Laser object.
     * @param serialRef Reference to the hardware serial port to use. Defaults to Serial.
     */
    SerialComms(Laser &laserRef, HardwareSerial &serialRef = Serial)
        : laser(&laserRef), serial(&serialRef)
    {
        // Initialize the specified Serial object, start it if it's not started
        serial->begin(115200);
        while (!serial)
            ; // Wait for the serial port to connect, necessary for some boards
    }

    /**
     * @brief Processes incoming serial data and executes commands as necessary.
     */
    void processIncomingData()
    {
        // Check if new data is available and handle accordingly
        if (Serial.available() > 0)
        {
            byte cmdByte = Serial.read(); // pop cmd byte
            if (cmdByte == static_cast<byte>(Command::StopProcessing))
            {
                laser->off();
                executeCommands = false;
                bufferSize = 0;
            }
            if (cmdByte == static_cast<byte>(Command::FrameStart))
            {
                laser->off();
                bufferSize = 0;
                Serial.readBytes((uint8_t *)&bufferSize, sizeof(uint16_t));
                Serial.println(bufferSize);
                if (bufferSize <= BUFFER_SIZE)
                {
                    Serial.readBytes((uint8_t *)&commandBuffer, bufferSize);
                    executeCommands = true;
                    Serial.flush();
                }
            }
        }
        else
        {
          if (bufferSize && executeCommands)
          {
            executeBufferedCommands();
          }
        }
    }

private:
    Laser *laser;                     ///< Pointer to the Laser object controlled by commands.
    bool hasValidCommand = false;     ///< Indicates if a valid command is being processed.
    byte commandBuffer[BUFFER_SIZE];  ///< Buffer to store incoming command data.
    uint16_t bufferSize = 0;          ///< Current size of the data in the command buffer.
    bool executeCommands = false;     ///< Flag to start executing buffered commands.
    HardwareSerial *serial;           ///< Hardware serial interface used for communication.

    /**
     * @brief Executes commands stored in the command buffer.
     */
    void executeBufferedCommands()
    {
        byte index = 0;
        while (index < bufferSize)
        {
            Command command = static_cast<Command>(commandBuffer[index++]);
            byte expectedPayloadSize = getPayloadSize(command);

            if (index + expectedPayloadSize <= bufferSize)
            {
                if (!executeCommand(command, &commandBuffer[index]))
                {
                  Serial.println("Unknown command");
                  return;
                }
                index += expectedPayloadSize;
            }
            else
            {
                Serial.println("Incomplete payload for command");
                break; // Stop processing on error
            }
        }
    }

    /**
     * @brief Generates a checksum for given data buffer. TODO: use this
     * @param buffer Pointer to data buffer.
     * @param size Size of the data buffer.
     * @return byte Computed checksum value.
     */
    byte generateChecksum(const byte *buffer, byte size)
    {
        byte checksum = 0;
        for (byte i = 0; i < size; ++i)
        {
            checksum += buffer[i];
        }
        return checksum;
    }

    /**
     * @brief Verifies the checksum of the command buffer. TODO: use this
     * @return bool True if the checksum is valid, false otherwise.
     */
    bool verifyChecksum()
    {
        byte checksum = 0;
        for (byte i = 1; i < bufferSize - 1; i++)
        { // Exclude the cmd bit and checksum byte itself
            checksum += commandBuffer[i];
        }
        return checksum == commandBuffer[bufferSize - 1];
    }

    /**
     * @brief Executes a single command using the provided payload.
     * @param command The command to execute.
     * @param payload Pointer to the payload associated with the command.
     * @return bool True if the command was successfully executed, false otherwise.
     */
    bool executeCommand(Command command, byte *payload)
    {
        switch (command)
        {
        case Command::TurnOn:
            laser->on();
            break;
        case Command::TurnOff:
            laser->off();
            break;
        case Command::SendTo:
        {
            // Extract x and y coordinates from the payload and move the laser
            uint16_t x = payload[0] | (payload[1] << 8);
            uint16_t y = payload[2] | (payload[3] << 8);
            laser->SendTo(x, y);
            break;
        }
        case Command::DrawLine:
        {
            // Extract coordinates x1, y1, x2, y2 from the payload and draw a line
            uint16_t x1 = payload[0] | (payload[1] << 8);
            uint16_t y1 = payload[2] | (payload[3] << 8);
            uint16_t x2 = payload[4] | (payload[5] << 8);
            uint16_t y2 = payload[6] | (payload[7] << 8);
            laser->drawLine(x1, y1, x2, y2);
            break;
        }
        case Command::DrawRect:
        {
            // Extract coordinates x1, y1, x2, y2 from the payload and draw a rectangle
            uint16_t rx1 = payload[0] | (payload[1] << 8);
            uint16_t ry1 = payload[2] | (payload[3] << 8);
            uint16_t rx2 = payload[4] | (payload[5] << 8);
            uint16_t ry2 = payload[6] | (payload[7] << 8);
            laser->drawRect(rx1, ry1, rx2, ry2);
            break;
        }
        case Command::SetColor:
        {
            // Extract RGB values from the payload and set the color
            laser->setColor({payload[0], payload[1], payload[2]});
            break;
        }
        case Command::Wait:
        {
            // Extract the duration from the payload and perform a wait
            uint16_t duration = payload[0] | (payload[1] << 8);
            delay(duration);
            break;
        }
        default:
            return false;
            break;
        }
        return true;
    }
};

#endif