std_msgs/Header header
uint64 rx_bytes
uint64 tx_bytes
uint32 valid_frames
uint32 cobs_errors
uint32 crc_errors
uint32 length_errors
uint32 version_errors
uint32 unknown_message_types
uint32 duplicate_sequences
uint32 out_of_order_sequences
uint32 oversized_frames
uint32 reconnects
uint32 handshake_failures

# Counters reported by the ESP32 DIAGNOSTICS payload. Keeping them separate
# avoids hiding link/parser failures on either side of the USB connection.
uint32 esp_rx_frames
uint32 esp_tx_frames
uint32 esp_crc_errors
uint32 esp_cobs_errors
uint32 esp_length_errors
uint32 esp_oversized_frames
uint32 esp_unknown_message_types
uint32 esp_duplicate_sequences
uint32 esp_out_of_order_sequences
uint32 esp_rx_overflows
uint32 esp_tx_errors
