#include "MessageBuffer.h"
#include "Debug.h"



MessageBuffer::MessageBuffer() {
  // Initialize all messages as empty
  for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
    messages[i].index      = i;
    messages[i].client_id  = 0;  // 0 indicates an empty slot
    messages[i].content[0] = 0;  // Empty string
  }
}

bool MessageBuffer::addMessage(uint8_t client_id, const char* content) {
  const char *debugMessage = "Buffer::addMessage client:%d message:%s %s";

  // Check if message for client_id already exists
  for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
    if (   messages[i].client_id == client_id
        && strcmp(messages[i].content, content) == 0) {
      // Message for this client_id already exists
      Debug::log(debugMessage, client_id, content, "already exists");
      return false;
    }
  }

  for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
    if (messages[i].client_id == 0) { // Find an empty slot
      messages[i].client_id = client_id;
      strncpy(messages[i].content, content, MESSAGE_SIZE - 1);
      messages[i].content[MESSAGE_SIZE - 1] = '\0'; // Ensure null-termination
      Debug::log(debugMessage, client_id, content, "added");
      return true;
    }
  }

  Debug::log(debugMessage, client_id, content, "buffer full");
  return false; // Buffer is full
}

bool MessageBuffer::getMessage(uint8_t client_id, Message *message) const {
  for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
    if (messages[i].client_id == client_id) {
      *message = messages[i];

      Debug::log("Buffer::getMessage client:%d message:%s", client_id, message->content);
      return true;
    }
  }

  return false;
}

void MessageBuffer::deleteMessage(Message *message) {
  if (message->index < BUFFER_SIZE) {
    messages[message->index].client_id = 0;   // Mark as empty
    messages[message->index].content[0] = 0;  // Clear content
  }
}

void MessageBuffer::dumpBuffer() const {
  Debug::log("MessageBuffer Dump:");
  for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
    if (messages[i].client_id != 0) {
      Debug::log(" Index: %d, Client ID: %d, Content: %s",
                 messages[i].index,
                 messages[i].client_id,
                 messages[i].content);
    }
  }
}
