#ifndef MESSAGEBUFFER_H
#define MESSAGEBUFFER_H

#include <Arduino.h>

class MessageBuffer {
  public:
    static const uint8_t BUFFER_SIZE  = 20;
    static const uint8_t MESSAGE_SIZE = 16;

    struct Message {
      uint8_t index;                // Index in the buffer array
      uint8_t client_id;            // Target client ID or 0 if not a valid message
      char content[MESSAGE_SIZE];   // Message content
    };

    MessageBuffer();

    /**
     * Adds a message to the buffer.
     * @param client_id The ID of the client which is the target for the message.
     * @param content The content of the message.
     * @return true if the message was added successfully, false if the buffer is full
     *         or the message for the client ID already exists.
     */
    bool addMessage(uint8_t client_id, const char* content);

    /**
     * Retrieves and removes a message for the specified client ID if available.
     * @param client_id The ID of the client to retrieve the message for.
     * @param message Pointer to a Message struct where the retrieved message will be stored.
     * @return true if a message was found and retrieved, false otherwise.
     */
    bool getMessage(uint8_t client_id, Message *message) const;

    /**
     * Deletes a message from the buffer.
     * @param message Pointer to the Message struct to be deleted.
     */
    void deleteMessage(Message *message);

    /**
     * dumps the current buffer contents to the serial interface for debugging
     */
    void dumpBuffer() const;
  
  private:
    Message messages[BUFFER_SIZE];
};
  
#endif // MESSAGEBUFFER_H
